#include <Arduino.h>
#include <Preferences.h>

/*
 ============================================================
 Projeto: Monitor de Lesão por Pressão (LPP)
 Plataforma: ESP32
 Sensores: 12 FSR + CD74HC4067
 Versão: 2.0
   - Correções na média móvel e no monitoramento por tempo
   - Calibração de zero e de máximo (salva na memória flash)
   - Detecção de possíveis falhas nos sensores e no MUX
   - Saída em texto ou JSON
 ============================================================

 Comandos pelo Monitor Serial (115200 baud, com "Nova linha"):
   z     -> calibra o zero (SEM peso sobre os sensores)
   m N   -> define o máximo do sensor N com a leitura atual
            (aplique a carga de referência antes). Ex.: m 7
   x     -> apaga a calibração salva e volta ao padrão
   j     -> alterna a saída entre texto e JSON
   s     -> mostra a calibração atual
   h     -> ajuda

 ATENÇÃO: os limiares (20%, 35%, 50%) são experimentais,
 não são limites clínicos validados. A porcentagem é do
 sinal calibrado, não uma medida de pressão em mmHg.
*/

//====================== CONFIGURAÇÕES ======================

#define NUM_SENSORS 12
#define MOVING_AVG_WINDOW 8
#define ADC_MAX 4095

// Período entre ciclos de leitura (ms)
#define PERIODO_LEITURA_MS 200

// Tempo de estabilização após trocar o canal do MUX (us)
#define MUX_SETTLE_US 100

// Saída padrão ao ligar (false = texto, true = JSON)
#define SAIDA_JSON_PADRAO false

// Multiplexador
#define MUX_S0 32
#define MUX_S1 19
#define MUX_S2 18
#define MUX_S3 23
#define ADC_PIN 34

// LEDs
#define LED_VERDE 25
#define LED_VERMELHO 26

// Limiares em porcentagem do sinal
#define LIMIAR_IMEDIATO 50
#define LIMIAR_MODERADO 35
#define LIMIAR_PROLONGADO 20

// Desativação do alerta imediato: ele liga com o maior valor
// >= 50% e só desliga quando o maior valor cai abaixo de 40%
#define LIMIAR_DESATIVACAO 40

// Histerese (pontos percentuais) para o contador de tempo
// não zerar quando a leitura oscila perto do limiar
#define HISTERESE 3

// Tempos para monitoramento
const unsigned long TEMPO_MODERADO =
    15UL * 60UL * 1000UL;

const unsigned long TEMPO_PROLONGADO =
    60UL * 60UL * 1000UL;

//--------------------- Calibração --------------------------

#define AMOSTRAS_CALIBRACAO 64

// Zero acima disso (~15% do ADC) indica sensor carregado
// durante a calibração ou com defeito
#define ZERO_MAX_ACEITAVEL 600

// Faixa útil mínima entre zero e máximo (contagens do ADC)
#define FAIXA_MINIMA 800

//------------------- Detecção de falhas --------------------

// Saturação: leitura no topo do ADC por tempo prolongado
// (possível curto ou divisor de tensão inadequado)
#define RAW_SATURADO 4050
const unsigned long TEMPO_SATURADO = 5000UL;

// Ruído: saltos bruscos entre leituras consecutivas
// (possível mau contato ou fio solto)
#define SALTO_RUIDO 1500
#define RUIDO_LIMITE 10

// Travado: sensor carregado sem nenhuma variação
// (o ADC do ESP32 sempre oscila um pouco)
const unsigned long JANELA_TRAVADO = 60000UL;
#define VARIACAO_MIN_TRAVADO 3

// MUX: todos os canais com o mesmo valor alto
#define MUX_TOLERANCIA 15
#define MUX_RAW_MIN 300

// Códigos de falha (bits)
#define FALHA_SATURADO   0x01
#define FALHA_RUIDO      0x02
#define FALHA_TRAVADO    0x04
#define FALHA_CALIBRACAO 0x08

//===========================================================

struct MovAvg {
  int buffer[MOVING_AVG_WINDOW];
  int idx;
  long sum;
  bool iniciado;
};

MovAvg filtros[NUM_SENSORS];

Preferences prefs;

// Associação ilustrativa dos sensores às regiões.
// Deve ser confirmada conforme a montagem física.
const char* regioes[NUM_SENSORS] = {
  "Occipital",
  "Orelha Esq",
  "Orelha Dir",
  "Escapula Esq",
  "Escapula Dir",
  "Cotovelo Esq",
  "Cotovelo Dir",
  "Sacro",
  "Trocanter Esq",
  "Trocanter Dir",
  "Calcaneo Esq",
  "Calcaneo Dir"
};

// Calibração
int calZero[NUM_SENSORS];
int calMax[NUM_SENSORS];

// Leituras do ciclo atual
int leituraRaw[NUM_SENSORS] = {0};
int leituraFiltrada[NUM_SENSORS] = {0};
int leituraPct[NUM_SENSORS] = {0};

// Monitoramento por tempo
bool emModerado[NUM_SENSORS] = {false};
bool emProlongado[NUM_SENSORS] = {false};

unsigned long inicioModerado[NUM_SENSORS] = {0};
unsigned long inicioProlongado[NUM_SENSORS] = {0};

bool alertaModerado[NUM_SENSORS] = {false};
bool alertaProlongado[NUM_SENSORS] = {false};

// Detecção de falhas
uint8_t falhas[NUM_SENSORS] = {0};
int ultimoRaw[NUM_SENSORS];
uint8_t contRuido[NUM_SENSORS] = {0};

bool emSaturado[NUM_SENSORS] = {false};
unsigned long inicioSaturado[NUM_SENSORS] = {0};

bool janelaTravado[NUM_SENSORS] = {false};
unsigned long inicioTravado[NUM_SENSORS] = {0};
int travMin[NUM_SENSORS] = {0};
int travMax[NUM_SENSORS] = {0};

bool falhaMux = false;

// Estado geral
bool estadoAlerta = false;
bool alertaImediato = false;
bool saidaJson = SAIDA_JSON_PADRAO;
bool piscaEstado = false;
unsigned long ultimaLeitura = 0;

// Comandos pela serial
char cmdBuf[16];
uint8_t cmdLen = 0;

//=================== FUNÇÕES DE SAÍDA ======================

// Mensagem simples, no formato da saída atual
void msg(const char* texto) {
  if (saidaJson) {
    Serial.print("{\"msg\":\"");
    Serial.print(texto);
    Serial.println("\"}");
  } else {
    Serial.println(texto);
  }
}

// Imprime um tempo em mm:ss
void imprimirTempo(unsigned long ms) {
  unsigned long s = ms / 1000UL;
  Serial.printf("%02lu:%02lu", s / 60UL, s % 60UL);
}

void imprimirFalhasTexto(uint8_t f) {
  if (f & FALHA_SATURADO)   Serial.print(" SATURADO");
  if (f & FALHA_RUIDO)      Serial.print(" RUIDO");
  if (f & FALHA_TRAVADO)    Serial.print(" TRAVADO");
  if (f & FALHA_CALIBRACAO) Serial.print(" CALIBRACAO");
}

void imprimirFalhasJson(uint8_t f) {
  bool primeiro = true;
  Serial.print("[");

  if (f & FALHA_SATURADO) {
    Serial.print("\"SATURADO\"");
    primeiro = false;
  }
  if (f & FALHA_RUIDO) {
    if (!primeiro) Serial.print(",");
    Serial.print("\"RUIDO\"");
    primeiro = false;
  }
  if (f & FALHA_TRAVADO) {
    if (!primeiro) Serial.print(",");
    Serial.print("\"TRAVADO\"");
    primeiro = false;
  }
  if (f & FALHA_CALIBRACAO) {
    if (!primeiro) Serial.print(",");
    Serial.print("\"CALIBRACAO\"");
  }

  Serial.print("]");
}

// Nível da leitura (usado no JSON)
const char* nivel(int pct) {
  if (pct >= LIMIAR_IMEDIATO) return "IMEDIATO";
  if (pct >= LIMIAR_MODERADO) return "MODERADO";
  if (pct >= LIMIAR_PROLONGADO) return "PROLONGADO";
  return "NORMAL";
}

// Rótulo da leitura (usado na saída em texto)
const char* rotulo(int pct) {
  if (pct >= LIMIAR_IMEDIATO) return "ALERTA IMEDIATO";
  if (pct >= LIMIAR_MODERADO) return "MONITORANDO 15min";
  if (pct >= LIMIAR_PROLONGADO) return "MONITORANDO 60min";
  return "NORMAL";
}

// Eventos de alerta e alívio
void imprimirEvento(const char* tipo, int id,
                    unsigned long duracaoMs) {
  if (saidaJson) {
    Serial.printf(
      "{\"evento\":\"%s\",\"id\":%d,\"regiao\":\"%s\","
      "\"duracao_s\":%lu}\n",
      tipo, id, regioes[id], duracaoMs / 1000UL);
  } else {
    Serial.print("\n*** ");
    Serial.print(tipo);
    Serial.println(" ***");
    Serial.print("Regiao: ");
    Serial.print(regioes[id]);
    Serial.print("  Tempo: ");
    imprimirTempo(duracaoMs);
    Serial.println();
  }
}

// Evento de mudança no estado de falha de um sensor
void imprimirEventoFalha(int id) {
  if (saidaJson) {
    Serial.printf(
      "{\"evento\":\"FALHA\",\"id\":%d,\"regiao\":\"%s\","
      "\"falhas\":",
      id, regioes[id]);
    imprimirFalhasJson(falhas[id]);
    Serial.println("}");
  } else {
    Serial.print("\n*** FALHA SENSOR ");
    Serial.print(id);
    Serial.print(" (");
    Serial.print(regioes[id]);
    Serial.print("):");
    if (falhas[id]) {
      imprimirFalhasTexto(falhas[id]);
    } else {
      Serial.print(" normalizado");
    }
    Serial.println(" ***");
  }
}

//====================== FUNÇÕES ============================

// Seleciona o canal do multiplexador
void muxSelect(byte ch) {
  digitalWrite(MUX_S0, ch & 1);
  digitalWrite(MUX_S1, (ch >> 1) & 1);
  digitalWrite(MUX_S2, (ch >> 2) & 1);
  digitalWrite(MUX_S3, (ch >> 3) & 1);
}

// Lê um canal do MUX.
// A primeira leitura é descartada porque o capacitor interno
// do ADC ainda pode guardar carga do canal anterior.
int lerCanal(byte ch) {
  muxSelect(ch);
  delayMicroseconds(MUX_SETTLE_US);

  analogRead(ADC_PIN);
  return analogRead(ADC_PIN);
}

// Inicializa o filtro
void movInit(MovAvg *f) {
  f->idx = 0;
  f->sum = 0;
  f->iniciado = false;

  for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
    f->buffer[i] = 0;
  }
}

// Atualiza a média móvel.
// Na primeira amostra o buffer é preenchido com ela,
// evitando leituras artificialmente baixas no início.
int movUpdate(MovAvg *f, int amostra) {
  if (!f->iniciado) {
    for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
      f->buffer[i] = amostra;
    }
    f->sum = (long)amostra * MOVING_AVG_WINDOW;
    f->idx = 0;
    f->iniciado = true;
    return amostra;
  }

  f->sum -= f->buffer[f->idx];

  f->buffer[f->idx] = amostra;
  f->sum += amostra;

  f->idx = (f->idx + 1) % MOVING_AVG_WINDOW;

  return f->sum / MOVING_AVG_WINDOW;
}

// Converte a leitura para porcentagem usando a calibração
int rawToPercent(int id, int raw) {
  int faixa = calMax[id] - calZero[id];

  // Proteção contra divisão por zero ou faixa inválida
  if (faixa < FAIXA_MINIMA) faixa = FAIXA_MINIMA;

  long pct = (long)(raw - calZero[id]) * 100L / faixa;

  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;

  return (int)pct;
}

// Controla os LEDs
//  alerta          -> vermelho fixo
//  falha sem alerta -> verde e vermelho alternando
//  normal          -> verde fixo
void atualizarLED(bool alerta, bool falha) {
  if (alerta) {
    digitalWrite(LED_VERDE, LOW);
    digitalWrite(LED_VERMELHO, HIGH);
  } else if (falha) {
    piscaEstado = !piscaEstado;
    digitalWrite(LED_VERDE, piscaEstado);
    digitalWrite(LED_VERMELHO, !piscaEstado);
  } else {
    digitalWrite(LED_VERDE, HIGH);
    digitalWrite(LED_VERMELHO, LOW);
  }
}

//====================== CALIBRAÇÃO =========================

void calibracaoPadrao() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    calZero[i] = 0;
    calMax[i] = ADC_MAX;
  }
}

// Marca os sensores com calibração suspeita
void validarCalibracao() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    bool invalida = calZero[i] > ZERO_MAX_ACEITAVEL ||
                    (calMax[i] - calZero[i]) < FAIXA_MINIMA;

    if (invalida) {
      falhas[i] |= FALHA_CALIBRACAO;
    } else {
      falhas[i] &= ~FALHA_CALIBRACAO;
    }
  }
}

void salvarCalibracao() {
  prefs.begin("lpp", false);
  prefs.putBytes("zero", calZero, sizeof(calZero));
  prefs.putBytes("max", calMax, sizeof(calMax));
  prefs.end();
}

bool carregarCalibracao() {
  size_t a = 0;
  size_t b = 0;

  if (prefs.begin("lpp", true)) {
    a = prefs.getBytes("zero", calZero, sizeof(calZero));
    b = prefs.getBytes("max", calMax, sizeof(calMax));
    prefs.end();
  }

  if (a != sizeof(calZero) || b != sizeof(calMax)) {
    calibracaoPadrao();
    return false;
  }

  return true;
}

// Zera filtros, tempos e contadores de falha
// (usado depois de qualquer mudança na calibração)
void reiniciarMonitoramento() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    movInit(&filtros[i]);

    emModerado[i] = false;
    emProlongado[i] = false;
    alertaModerado[i] = false;
    alertaProlongado[i] = false;

    falhas[i] &= FALHA_CALIBRACAO;
    ultimoRaw[i] = -1;
    contRuido[i] = 0;
    emSaturado[i] = false;
    janelaTravado[i] = false;
  }

  falhaMux = false;
}

// Calibração do zero: mede cada sensor sem carga
void calibrarZero() {
  msg("Calibrando zero... mantenha os sensores SEM peso.");

  long soma[NUM_SENSORS] = {0};

  for (int a = 0; a < AMOSTRAS_CALIBRACAO; a++) {
    for (int i = 0; i < NUM_SENSORS; i++) {
      soma[i] += lerCanal(i);
    }
    delay(5);
  }

  for (int i = 0; i < NUM_SENSORS; i++) {
    calZero[i] = soma[i] / AMOSTRAS_CALIBRACAO;
  }

  salvarCalibracao();
  validarCalibracao();
  reiniciarMonitoramento();

  msg("Zero calibrado e salvo.");

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (falhas[i] & FALHA_CALIBRACAO) {
      imprimirEventoFalha(i);
    }
  }
}

// Calibração do máximo de um sensor com a leitura atual
void calibrarMaximo(int id) {
  int valor = leituraFiltrada[id];

  if (valor - calZero[id] < FAIXA_MINIMA) {
    msg("Leitura muito baixa para definir o maximo. "
        "Aplique a carga de referencia e tente de novo.");
    return;
  }

  calMax[id] = valor;

  salvarCalibracao();
  validarCalibracao();

  if (saidaJson) {
    Serial.printf(
      "{\"msg\":\"maximo calibrado\",\"id\":%d,\"max\":%d}\n",
      id, valor);
  } else {
    Serial.printf("Maximo do sensor %d (%s) = %d\n",
                  id, regioes[id], valor);
  }
}

void apagarCalibracao() {
  prefs.begin("lpp", false);
  prefs.clear();
  prefs.end();

  calibracaoPadrao();
  validarCalibracao();
  reiniciarMonitoramento();

  msg("Calibracao apagada. Usando valores padrao.");
}

void mostrarCalibracao() {
  if (saidaJson) {
    Serial.print("{\"calibracao\":[");
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (i > 0) Serial.print(",");
      Serial.printf(
        "{\"id\":%d,\"regiao\":\"%s\",\"zero\":%d,"
        "\"max\":%d,\"valida\":%s}",
        i, regioes[i], calZero[i], calMax[i],
        (falhas[i] & FALHA_CALIBRACAO) ? "false" : "true");
    }
    Serial.println("]}");
  } else {
    Serial.println("\n------ CALIBRACAO ------");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.printf("%2d %-14s zero=%4d max=%4d%s\n",
                    i, regioes[i], calZero[i], calMax[i],
                    (falhas[i] & FALHA_CALIBRACAO)
                      ? "  [INVALIDA]" : "");
    }
    Serial.println("------------------------");
  }
}

//================= DETECÇÃO DE FALHAS ======================

void detectarFalhas(int id, int raw, int pct,
                    unsigned long agora) {

  // Saturação: leitura no topo do ADC por muito tempo
  if (raw >= RAW_SATURADO) {
    if (!emSaturado[id]) {
      emSaturado[id] = true;
      inicioSaturado[id] = agora;
    }
    if ((agora - inicioSaturado[id]) >= TEMPO_SATURADO) {
      falhas[id] |= FALHA_SATURADO;
    }
  } else {
    emSaturado[id] = false;
    falhas[id] &= ~FALHA_SATURADO;
  }

  // Ruído: contador que sobe com saltos bruscos
  // e desce lentamente quando a leitura é estável
  if (ultimoRaw[id] >= 0) {
    if (abs(raw - ultimoRaw[id]) > SALTO_RUIDO) {
      contRuido[id] += 2;
      if (contRuido[id] > 2 * RUIDO_LIMITE) {
        contRuido[id] = 2 * RUIDO_LIMITE;
      }
    } else if (contRuido[id] > 0) {
      contRuido[id]--;
    }
  }
  ultimoRaw[id] = raw;

  if (contRuido[id] >= RUIDO_LIMITE) {
    falhas[id] |= FALHA_RUIDO;
  } else if (contRuido[id] == 0) {
    falhas[id] &= ~FALHA_RUIDO;
  }

  // Travado: sensor carregado sem variação nenhuma
  if (pct >= LIMIAR_PROLONGADO) {
    if (!janelaTravado[id]) {
      janelaTravado[id] = true;
      inicioTravado[id] = agora;
      travMin[id] = raw;
      travMax[id] = raw;
    } else {
      if (raw < travMin[id]) travMin[id] = raw;
      if (raw > travMax[id]) travMax[id] = raw;

      if ((agora - inicioTravado[id]) >= JANELA_TRAVADO) {
        if ((travMax[id] - travMin[id]) < VARIACAO_MIN_TRAVADO) {
          falhas[id] |= FALHA_TRAVADO;
        } else {
          falhas[id] &= ~FALHA_TRAVADO;
        }
        janelaTravado[id] = false;  // começa nova janela
      }
    }
  } else {
    janelaTravado[id] = false;
    falhas[id] &= ~FALHA_TRAVADO;
  }
}

// Possível falha no MUX: todos os canais com o mesmo
// valor alto (ex.: pino de seleção ou SIG em curto)
void verificarMux() {
  int mn = leituraRaw[0];
  int mx = leituraRaw[0];

  for (int i = 1; i < NUM_SENSORS; i++) {
    if (leituraRaw[i] < mn) mn = leituraRaw[i];
    if (leituraRaw[i] > mx) mx = leituraRaw[i];
  }

  bool anterior = falhaMux;
  falhaMux = (mx - mn) <= MUX_TOLERANCIA && mn >= MUX_RAW_MIN;

  if (falhaMux != anterior) {
    msg(falhaMux
          ? "*** POSSIVEL FALHA NO MULTIPLEXADOR ***"
          : "Multiplexador normalizado.");
  }
}

//================= MONITORAMENTO POR TEMPO =================

// O tempo moderado conta enquanto pct >= 35% (inclusive na
// faixa imediata) e o prolongado enquanto pct >= 20%.
// Assim, um pico acima do limiar não zera o tempo acumulado.
void verificarTempos(int id, int pct, unsigned long agora) {

  // Nível moderado
  bool acimaMod = emModerado[id]
                    ? pct >= (LIMIAR_MODERADO - HISTERESE)
                    : pct >= LIMIAR_MODERADO;

  if (acimaMod) {
    if (!emModerado[id]) {
      emModerado[id] = true;
      inicioModerado[id] = agora;
    }

    if ((agora - inicioModerado[id]) >= TEMPO_MODERADO &&
        !alertaModerado[id]) {

      alertaModerado[id] = true;
      imprimirEvento("ALERTA MODERADO", id,
                     agora - inicioModerado[id]);
    }

  } else {
    if (alertaModerado[id]) {
      imprimirEvento("ALIVIO MODERADO", id,
                     agora - inicioModerado[id]);
    }
    emModerado[id] = false;
    alertaModerado[id] = false;
  }

  // Nível prolongado
  bool acimaProl = emProlongado[id]
                     ? pct >= (LIMIAR_PROLONGADO - HISTERESE)
                     : pct >= LIMIAR_PROLONGADO;

  if (acimaProl) {
    if (!emProlongado[id]) {
      emProlongado[id] = true;
      inicioProlongado[id] = agora;
    }

    if ((agora - inicioProlongado[id]) >= TEMPO_PROLONGADO &&
        !alertaProlongado[id]) {

      alertaProlongado[id] = true;
      imprimirEvento("ALERTA PROLONGADO", id,
                     agora - inicioProlongado[id]);
    }

  } else {
    if (alertaProlongado[id]) {
      imprimirEvento("ALIVIO PROLONGADO", id,
                     agora - inicioProlongado[id]);
    }
    emProlongado[id] = false;
    alertaProlongado[id] = false;
  }
}

//====================== SAÍDAS DO CICLO ====================

void imprimirTexto(int maior, int sensorMaior,
                   bool algumaFalha, unsigned long agora) {
  Serial.println("\n------------------------------");

  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.printf("%2d %-14s -> %3d%%  [%s]",
                  i, regioes[i], leituraPct[i],
                  rotulo(leituraPct[i]));

    if (emModerado[i]) {
      Serial.print("  t35=");
      imprimirTempo(agora - inicioModerado[i]);
    }
    if (emProlongado[i]) {
      Serial.print("  t20=");
      imprimirTempo(agora - inicioProlongado[i]);
    }
    if (alertaModerado[i])   Serial.print("  !MOD");
    if (alertaProlongado[i]) Serial.print("  !PROL");

    if (falhas[i]) {
      Serial.print("  FALHA:");
      imprimirFalhasTexto(falhas[i]);
    }

    Serial.println();
  }

  Serial.println("------------------------------");

  Serial.print("Maior valor: ");
  Serial.print(maior);
  Serial.println("%");

  if (sensorMaior >= 0) {
    Serial.print("Regiao critica: ");
    Serial.println(regioes[sensorMaior]);
  }

  if (falhaMux) {
    Serial.println("MUX: POSSIVEL FALHA");
  }

  Serial.print("LED: ");
  if (estadoAlerta) {
    Serial.println("VERMELHO");
  } else if (algumaFalha) {
    Serial.println("PISCANDO (falha)");
  } else {
    Serial.println("VERDE");
  }
}

void imprimirJson(int maior, int sensorMaior,
                  bool algumaFalha, unsigned long agora) {
  Serial.printf("{\"t_ms\":%lu,\"alerta\":%s,"
                "\"alerta_imediato\":%s,\"falha\":%s,"
                "\"falha_mux\":%s,",
                agora,
                estadoAlerta ? "true" : "false",
                alertaImediato ? "true" : "false",
                algumaFalha ? "true" : "false",
                falhaMux ? "true" : "false");

  if (sensorMaior >= 0) {
    Serial.printf("\"maior\":{\"pct\":%d,\"id\":%d,"
                  "\"regiao\":\"%s\"},",
                  maior, sensorMaior, regioes[sensorMaior]);
  } else {
    Serial.print("\"maior\":null,");
  }

  Serial.print("\"sensores\":[");

  for (int i = 0; i < NUM_SENSORS; i++) {
    if (i > 0) Serial.print(",");

    unsigned long tMod = emModerado[i]
                           ? (agora - inicioModerado[i]) / 1000UL
                           : 0;
    unsigned long tProl = emProlongado[i]
                            ? (agora - inicioProlongado[i]) / 1000UL
                            : 0;

    Serial.printf(
      "{\"id\":%d,\"regiao\":\"%s\",\"raw\":%d,"
      "\"filtrado\":%d,\"pct\":%d,\"nivel\":\"%s\","
      "\"t_mod_s\":%lu,\"t_prol_s\":%lu,"
      "\"alerta_mod\":%s,\"alerta_prol\":%s,\"falhas\":",
      i, regioes[i], leituraRaw[i], leituraFiltrada[i],
      leituraPct[i], nivel(leituraPct[i]), tMod, tProl,
      alertaModerado[i] ? "true" : "false",
      alertaProlongado[i] ? "true" : "false");

    imprimirFalhasJson(falhas[i]);
    Serial.print("}");
  }

  Serial.println("]}");
}

//====================== COMANDOS ===========================

void mostrarAjuda() {
  msg("Comandos: z=calibra zero | m N=maximo do sensor N | "
      "x=apaga calibracao | j=texto/JSON | s=calibracao | "
      "h=ajuda");
}

void executarComando(char* cmd) {
  switch (tolower(cmd[0])) {
    case 'z':
      calibrarZero();
      break;

    case 'm': {
      char* fim;
      long id = strtol(cmd + 1, &fim, 10);

      if (fim == cmd + 1 || id < 0 || id >= NUM_SENSORS) {
        msg("Uso: m N  (N de 0 a 11). Ex.: m 7");
      } else {
        calibrarMaximo((int)id);
      }
      break;
    }

    case 'x':
      apagarCalibracao();
      break;

    case 'j':
      saidaJson = !saidaJson;
      msg(saidaJson ? "Saida JSON ativada."
                    : "Saida em texto ativada.");
      break;

    case 's':
      mostrarCalibracao();
      break;

    case 'h':
      mostrarAjuda();
      break;

    default:
      msg("Comando desconhecido. Envie h para ajuda.");
      break;
  }
}

// Lê comandos sem bloquear o loop
void processarSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = '\0';
        executarComando(cmdBuf);
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

//====================== SETUP ==============================

void setup() {
  Serial.begin(115200);

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);

  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_VERMELHO, OUTPUT);

  analogReadResolution(12);

  bool calibrado = carregarCalibracao();
  validarCalibracao();
  reiniciarMonitoramento();

  atualizarLED(false, false);

  if (!saidaJson) {
    Serial.println("\n====================================");
    Serial.println(" MONITOR DE LESAO POR PRESSAO v2.0");
    Serial.println("====================================");
  }

  if (calibrado) {
    msg("Calibracao carregada da memoria.");
  } else {
    msg("Sem calibracao salva. Envie z com os sensores "
        "sem peso para calibrar o zero.");
  }

  mostrarAjuda();
}

//======================= LOOP ==============================

void loop() {
  processarSerial();

  unsigned long agora = millis();

  if ((agora - ultimaLeitura) < PERIODO_LEITURA_MS) {
    return;
  }
  ultimaLeitura = agora;

  int maior = 0;
  int sensorMaior = -1;
  bool algumAlertaTempo = false;
  bool algumaFalha = false;

  for (int i = 0; i < NUM_SENSORS; i++) {
    int raw = lerCanal(i);

    leituraRaw[i] = raw;
    leituraFiltrada[i] = movUpdate(&filtros[i], raw);
    leituraPct[i] = rawToPercent(i, leituraFiltrada[i]);

    uint8_t falhasAntes = falhas[i];
    detectarFalhas(i, raw, leituraPct[i], agora);

    if (falhas[i] != falhasAntes) {
      imprimirEventoFalha(i);
    }

    verificarTempos(i, leituraPct[i], agora);

    if (alertaModerado[i] || alertaProlongado[i]) {
      algumAlertaTempo = true;
    }

    if (falhas[i]) {
      algumaFalha = true;
    }

    if (leituraPct[i] > maior) {
      maior = leituraPct[i];
      sensorMaior = i;
    }
  }

  verificarMux();
  if (falhaMux) algumaFalha = true;

  // Alerta imediato com histerese:
  // liga com maior >= 50% e desliga só com maior < 40%,
  // evitando que o LED fique piscando perto do limite.
  if (!alertaImediato && maior >= LIMIAR_IMEDIATO) {
    alertaImediato = true;
  } else if (alertaImediato && maior < LIMIAR_DESATIVACAO) {
    alertaImediato = false;
  }

  // Alerta geral: pressão imediata OU alerta por tempo.
  // Sensores com falha continuam sendo considerados
  // (é mais seguro alarmar à toa do que silenciar).
  estadoAlerta = alertaImediato || algumAlertaTempo;

  atualizarLED(estadoAlerta, algumaFalha);

  if (saidaJson) {
    imprimirJson(maior, sensorMaior, algumaFalha, agora);
  } else {
    imprimirTexto(maior, sensorMaior, algumaFalha, agora);
  }
}
