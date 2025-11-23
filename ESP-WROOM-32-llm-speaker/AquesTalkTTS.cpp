/*************************************************
  AquesTalkTTS - AquesTalk ESP32のラッパークラス
  漢字仮名混じりテキストからの音声合成も可能

  特徴：
  ・I2S出力。外付けのI2SアンプやI2S DACが別途必要
  ・非同期型で、別タスクで発声処理
  ・SDのアクセスは、SD.hの代わりに高速なSdFat.h(SdFatライブラリ)の指定も可能

  動作確認環境：
     ESP32-DevKitC(ESP32), M5StampS3(ESP32S3),
     Arduino core for the ESP32 v2.0.17, v3.0.5
     SdFat v2.2.3

  * ESP32, ESP32-S2, ESP32-S3 用です。
  * SPIやSDのピン番号は、使用するハードウェア環境に応じて変更してください。
  * I2S出力フォーマット: I2S標準(Philips Format), 16bit, LEFT_ONLY, 8KHzサンプリング
  * 音声記号列からの合成だけの場合は、辞書データやSD周りの設定は不要です
  * AquesTalk ESP32ライブラリはあらかじめzipファイルからインストールしてください
    https://www.a-quest.com/download.html
  * aqdic_open()/aqdic_close()/aqdic_read()関数は 辞書データ(aqdic_m.bin)にアクセスする関数で、
    libaquestalk.aから呼び出されます。
  * 漢字テキストからの合成の場合は、辞書データファイルをSDカード上に配置しておきます。
  * ワークバッファを実行時にヒープ上に確保します。create()は400B、createK()は20.4KBを使います。
  * 本ソースは改変自由です
**************************************************/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_system.h"

// #define USE_FATFS // FatFsを用いる場合に指定
// #define USE_SDFAT // SdFatライブラリ(別途インストール）を用いる場合に指定

#if defined(USE_FATFS)
#include <FFat.h>
#elif defined(USE_SDFAT)
#include <SdFat.h>
#else
#include <SD.h>
#endif
#include <aquestalk.h>
#include "AquesTalkTTS.h"

// AquesTalk-ESP32 licencekey ナ行マ行がヌになる制限を解除
static constexpr char *LICENCE_KEY = "XXX-XXX-XXX";

// -------------------------------------
// I2S pin  実際の配線に応じた設定が必要
// -------------------------------------

// ESP32-DevKitC
static constexpr gpio_num_t I2S_BCK_IO = GPIO_NUM_5;
static constexpr gpio_num_t I2S_WS_IO = GPIO_NUM_25;
static constexpr gpio_num_t I2S_DO_IO = GPIO_NUM_26;

// M5StampS3
// static constexpr gpio_num_t I2S_BCK_IO  = GPIO_NUM_9;
// static constexpr gpio_num_t I2S_WS_IO   = GPIO_NUM_43;
// static constexpr gpio_num_t I2S_DO_IO   = GPIO_NUM_44;

// -------------------------------------
// SD pin  実際の配線に応じた設定が必要
// -------------------------------------

// ESP32-DevKitC
static constexpr gpio_num_t SDCARD_CSPIN = GPIO_NUM_4;
static constexpr gpio_num_t SDCARD_DIPIN = GPIO_NUM_23;
static constexpr gpio_num_t SDCARD_DOPIN = GPIO_NUM_19;
static constexpr gpio_num_t SDCARD_SCKPIN = GPIO_NUM_18;

// M5StampS3
// static constexpr gpio_num_t SDCARD_CSPIN = GPIO_NUM_1;
// static constexpr gpio_num_t SDCARD_DIPIN = GPIO_NUM_3;
// static constexpr gpio_num_t SDCARD_DOPIN = GPIO_NUM_5;
// static constexpr gpio_num_t SDCARD_SCKPIN = GPIO_NUM_7;

static constexpr const char *FILE_DIC = "/aq_dic/aqdic_m.bin"; // SD上の辞書データの場所
static constexpr int LEN_FRAME = 32;
static constexpr int N_BUF_KOE = 1024; // 音声記号列のバッファサイズ
static constexpr i2s_port_t I2S_NUM = I2S_NUM_0;

static uint32_t *workbuf = nullptr; // work buffer for AquesTalk pico
static uint8_t *workbufK = nullptr; // work buffer for AqKanji2Roman
static TaskHandle_t task_handle = nullptr;
static volatile bool is_talking = false;
static int level = 0;      // フレーム単位の音量(リップシンク用)
static uint8_t gVol = 255; // 音量 初期値はmax

static void talk_task(void *arg);
static int calc_level(int16_t *wav, uint16_t len);
static void volume(int16_t *wav, uint16_t len, uint8_t gVol);
static void I2S_setup();

AquesTalkTTS TTS; // the only instance of AquesTalkTTS class

// 漢字仮名混じり文からの音声合成の場合の初期化 heap:21KB
int AquesTalkTTS::createK()
{
  if (!sd_begin())
    return 501; // ERR:SDカード未検出

  if (!workbufK)
  {
    workbufK = (uint8_t *)malloc(SIZE_AQK2R_MIN_WORK_BUF);
    if (workbufK == 0)
      return 401; // no heap memory
  }
  int iret = CAqK2R_Create(workbufK, SIZE_AQK2R_MIN_WORK_BUF);
  if (iret)
  {
    free(workbufK);
    workbufK = 0;
    return iret; // AqKanji2Roman Init error
  }
  return create(); // 音声記号からの音声合成の初期化
}

// 音声記号からの音声合成のみを使う場合の初期化 heap:400B
int AquesTalkTTS::create()
{
  I2S_setup();

  if (!workbuf)
  {
    workbuf = (uint32_t *)malloc(AQ_SIZE_WORKBUF * sizeof(uint32_t));
    if (workbuf == 0)
      return 401; // no heap memory
  }
  int iret = CAqTkPicoF_Init(workbuf, LEN_FRAME, LICENCE_KEY);
  if (iret)
    return iret; // AquesTalk Init error

  xTaskCreateUniversal(talk_task, "talk_task", 4096, nullptr, 3, &task_handle, APP_CPU_NUM);
  return 0;
}

void AquesTalkTTS::release()
{
  stop();
  if (task_handle)
    vTaskDelete(task_handle);
  if (workbuf)
    free(workbuf);
  if (workbufK)
  {
    CAqK2R_Release();
    free(workbufK);
  }
  workbuf = 0;
  workbufK = 0;
  task_handle = nullptr;
}

int AquesTalkTTS::playK(const char *kanji, int speed)
{
  if (workbufK == 0)
    return 403; // not initialized ( use TTS.createK() before. )
  if (workbuf == 0)
    return 402; // not initialized use TTS.create()

  // 漢字テキストを音声記号列に変換
  char koe[N_BUF_KOE];
  int iret = CAqK2R_Convert(kanji, koe, N_BUF_KOE);
  if (iret)
    return iret + 1000; // return error code

  return play(koe, speed); // 音声記号列からの音声合成 非同期
}

int AquesTalkTTS::play(const char *koe, int speed)
{
  stop();
  int iret = CAqTkPicoF_SetKoe((const uint8_t *)koe, 100, 0xFFu);
  if (iret)
    return iret; // return error code

  is_talking = true;
  xTaskNotifyGive(task_handle);
  return 0;
}

void AquesTalkTTS::stop()
{
  if (is_talking)
  {
    is_talking = false;
    vTaskDelay(1);
  }
}

bool AquesTalkTTS::isPlay()
{
  return is_talking;
}

void AquesTalkTTS::wait()
{
  while (is_talking)
  {
    vTaskDelay(1);
  }
}

int AquesTalkTTS::getLevel()
{
  return level;
}

void AquesTalkTTS::setVolume(uint8_t vol)
{
  gVol = vol;
}
//-----------------------------------------------------------------
// 逐次発声用タスク
void talk_task(void *arg)
{
  int16_t wav[LEN_FRAME];
  for (;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait notify
    while (is_talking)
    {
      uint16_t len;
      if (CAqTkPicoF_SyntheFrame(wav, &len))
      {
        is_talking = false;
        break;
      }
      level = calc_level(wav, len); // calc output level for Avatar(lip-sync)
      volume(wav, len, gVol);
      size_t i2s_bytes_write;
      i2s_write(I2S_NUM, wav, len * sizeof(uint16_t), &i2s_bytes_write, portMAX_DELAY);
    }
  }
}
// calc output level for Avatar(lip-sync)
int calc_level(int16_t *wav, uint16_t len)
{
  uint32_t sum = 0;
  if (len != LEN_FRAME)
    return 0;
  for (uint16_t i = 0; i < LEN_FRAME; i++)
  {
    sum += abs(wav[i]);
  }
  return (int)(sum / LEN_FRAME); // 5bit right shift
}

void volume(int16_t *wav, uint16_t len, uint8_t gVol)
{
  if (gVol == 255)
    return;
  for (uint16_t i = 0; i < len; i++)
  {
    wav[i] = (int16_t)(wav[i] * gVol / 256);
  }
}

/*****************************************************************************

  I2Sの初期化

  I2S_NUMやGPIOのピン番号は実装に合わせる必要があります。

******************************************************************************/

// AquesTalk ESP32　出力データのサンプリング周波数は8KHz固定
static constexpr uint32_t SAMPLE_RATE = 8000;

void I2S_setup()
{
  i2s_config_t i2s_config = {
      .mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_TX), // Only TX
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      //        .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // Interrupt level 1
      .dma_buf_count = 3,
      .dma_buf_len = LEN_FRAME, // one cycle per one buffer
      .use_apll = false,
  };
  i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_BCK_IO,
      .ws_io_num = I2S_WS_IO,
      .data_out_num = I2S_DO_IO,
      .data_in_num = GPIO_NUM_NC // not connect
  };
  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
  i2s_set_clk(I2S_NUM, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO); // 16bits, 1 channels
}

/*****************************************************************************

  辞書データ(aqdic_m.bin)のアクセス関数

  ここで定義する関数は、AquesTalk ESP32ライブラリから呼び出されます。
  辞書データを読み込む機能を、使用するハードウェア構成に応じて実装します。

  漢字仮名混じり文からの音声合成を行う場合に記述が必須で、
  さもなければリンク時にエラーとなります。
  音声記号列からの音声合成だけを使用する場合はこれら関数の記述は不要です。

  辞書データの配置場所は以下などが考えられます。
  ・SDカード上のファイル  << 本実装
  ・SPIシリアルフラッシュメモリ
  ・メモリマップドされたシリアルフラッシュ
  ・マイコン内蔵フラッシュメモリ

  辞書データは大量かつランダムにアクセスされるので、
  この関数の処理量が音声合成のパフォーマンスに与える影響は大きいです。

******************************************************************************/
//-----------------------------------------------------------------
// FileSystemの起動
//    エラーのときはtrueを返す
#if defined(USE_FATFS)
static File fp;
bool AquesTalkTTS::sd_begin()
{
  return FFat.begin();
}
#elif defined(USE_SDFAT)
static SdFs sd;
static FsFile fp;
bool AquesTalkTTS::sd_begin()
{
  SPI.begin(SDCARD_SCKPIN, SDCARD_DOPIN, SDCARD_DIPIN, SDCARD_CSPIN);
  return sd.begin(SdSpiConfig(SDCARD_CSPIN, SHARED_SPI, SD_SCK_MHZ(24)));
}
#else
static File fp = (File) nullptr;
bool AquesTalkTTS::sd_begin()
{
  SPI.begin(SDCARD_SCKPIN, SDCARD_DOPIN, SDCARD_DIPIN, SDCARD_CSPIN);
  return SD.begin(SDCARD_CSPIN, SPI, 24000000);
}

#endif

// 仮想的な辞書データの先頭アドレス （NULL以外なら任意で良い。但し4byteアライメント）
static uint16_t ADDR_ORG = 0x10001000U;

//-----------------------------------------------------------------
// 辞書データアクセスの初期化
// CAqK2R_Create()内から一度だけ呼び出される
// 戻り値
//    仮想的な辞書データの先頭アドレスを返す（0以外。4byteアライメント)。
//    エラーのときは0を返す
extern "C" size_t aqdic_open()
{
#if defined(USE_FATFS)
  fp = FFat.open(FILE_DIC);
#elif defined(USE_SDFAT)
  fp = sd.open(FILE_DIC);
#else
  fp = SD.open(FILE_DIC);
#endif
  if (!fp)
    return 0;      // err
  return ADDR_ORG; // ok
}

//-----------------------------------------------------------------
// 辞書データアクセスの終了
// CAqK2R_Release()内から一度だけ呼び出される
extern "C" void aqdic_close()
{
  if (fp)
    fp.close();
}

//-----------------------------------------------------------------
// 辞書データの読み込み
// pos: 先頭アドレス[byte]
// size: 読み込むサイズ[byte]
// buf:  読み込むデータ配列 uint8_t(size)
// 戻り値： 読みこんだバイト数
// CAqK2R_Convert()/CAqK2R_ConvertW()から複数回呼び出される
extern "C" size_t aqdic_read(size_t pos, size_t size, void *buf)
{
  fp.seek(pos - ADDR_ORG);
  return fp.read((uint8_t *)buf, size);
}
