/* USER CODE BEGIN Header */
/**
 ==========================================================================

		Projeto 2

		Disciplina: EA801- Laboratório Projetos Sistemas Embarcados

		Título do projeto:
		SISTEMA DE CONTROLE PWM PARA ILUMINAÇÃO
        LED E CLIMATIZAÇÃO EM AGRICULTURA INDOOR

		Alunos (RA):
		Davi Alves Feitosa de Souza (256447)
		Gabriel Martins De Andrade (216337)

		Docentes: Antônio Augusto Fasolo Quevedo

		Data: 16 de Maio, 2025

==========================================================================
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"  // Arquivo principal de cabeçalho do projeto

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>           // Biblioteca padrão para funções de entrada/saída
#include <string.h>          // Biblioteca padrão para manipulação de strings
#include "ssd1306.h"         // Biblioteca para controle do display OLED SSD1306
#include "ssd1306_fonts.h"   // Fontes utilizadas no display OLED
#include "stdio.h"           // Repetição desnecessária, mas sem prejuízo funcional
#include "ssd1306_conf.h"    // Configurações específicas do SSD1306
#include "core_cm4.h"        // Instruções de controle para ARM Cortex-M4
#include "stm32f4xx_hal_dma.h" // Biblioteca da HAL para uso de DMA com STM32F4
#include "ws2812_spi.h"      // Biblioteca personalizada para controle dos LEDs WS2812 via SPI
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Tipos definidos pelo usuário (nenhum declarado neste trecho)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_LEDS     25   // Número total de LEDs WS2812 na matriz
#define DEBOUNCE_MS  5    // Tempo de debounce para botões, em milissegundos
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Macros definidos pelo usuário (nenhum declarado neste trecho)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;    // Manipulador do ADC1

I2C_HandleTypeDef hi2c1;    // Manipulador da interface I2C1 (usada pelo display OLED)
I2C_HandleTypeDef hi2c2;    // Manipulador da interface I2C2 (possível uso para sensor)

SPI_HandleTypeDef hspi5;    // Manipulador da interface SPI5 (usada com WS2812)

TIM_HandleTypeDef htim2;    // Manipulador do temporizador 2
TIM_HandleTypeDef htim3;    // Manipulador do temporizador 3


/* USER CODE BEGIN PV */

// Buffer para armazenar as últimas leituras do sensor SHT-20
static float sht_temp, sht_hum;

// Matriz de cores dos LEDs WS2812, no formato GRB
static uint8_t ws2812_rgb[ NUM_LEDS ][3];

// Níveis de intensidade luminosa em % do LED mapeado em: 0%, 10%, 35%, 70%, 100%)
static const uint8_t intensity_levels[5] = { 0, 10, 35, 70, 100 };


static const uint8_t compositions[4][3] = {
  {30, 40, 30},	// White     = 40%R, 30%G, 30%B
  { 0, 55, 45},	// Pink      = 55%R,  0%G, 45%B
  {10, 88,  2},	// Red+Green = 88%R, 10%G,  2%B
  { 1,  1,  1} 	// Full spectrum (todos iguais) = 100%R, 100%G, 100%B
};

static uint8_t intensity_idx   = 2;  // Índice atual da intensidade (35%)
static uint8_t composition_idx = 3;  // Índice atual da composição (Full)

// Flags para indicar se os botões A e B foram pressionados
static uint8_t btnA_pressed = 0;
static uint8_t btnB_pressed = 0;

// Variáveis para leitura com debounce do botão A
static GPIO_PinState lastRawA = GPIO_PIN_RESET;
static uint32_t      lastDebounceTimeA = 0;

// Variáveis para leitura com debounce do botão B
static GPIO_PinState lastRawB = GPIO_PIN_RESET;
static uint32_t      lastDebounceTimeB = 0;

// Estado da interface de usuário (UI)
static uint8_t display_page    = 0;        // Página atual do display (0 = Home, 1 = Debug)
static GPIO_PinState lastRawSw = GPIO_PIN_RESET;
static uint32_t      lastDebounceTimeSw = 0;
static uint8_t       sw_pressed  = 0;      // Flag para pressionamento do joystick/switch

// Offsets para simulação de temperatura e umidade
static int8_t temp_offset  = 0;            // Offset da temperatura (-25 a +25)
static int8_t hum_offset   = 0;            // Offset da umidade (-50 a +50)

// Rótulos de texto para os níveis de intensidade exibidos no display
static const char *intensity_labels[5] = {
  "Desligado", "Minimo", "Medio", "Alto", "Maximo"
};

// Rótulos de texto para as composições de cor exibidas no display
static const char *composition_labels[4] = {
  "White", "Pink", "Red+Green", "Full"
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void MX_I2C1_Init(void);         // Inicialização do barramento I2C1
void SystemClock_Config(void);          // Configuração do clock do sistema
static void MX_GPIO_Init(void);         // Inicialização dos pinos GPIO
static void MX_ADC1_Init(void);         // Inicialização do conversor analógico-digital
static void MX_TIM3_Init(void);         // Inicialização do timer 3
static void MX_I2C2_Init(void);         // Inicialização do barramento I2C2
static void MX_SPI5_Init(void);         // Inicialização da interface SPI5
static void MX_TIM2_Init(void);         // Inicialização do timer 2
static void check_switch(void);         // Função para verificar estado do botão joystick
static void update_offsets(void);       // Atualiza os offsets de temperatura e umidade
static void display_home(void);         // Atualiza a interface da página Home
static void update_matrix(void);        // Atualiza a matriz de LEDs com base nos índices ativos
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Função ler_adc() ----------------------------------------------------------------------

Objetivo: Verifica o estado dos botões A (PB12) e B (PA15), aplicando
          debounce e atualizando índices globais conforme cliques.

Como funciona:
 	1. Cria e configura a estrutura sConfig com o canal desejado.
 	2. Define o tempo de amostragem (15 ciclos de clock).
 	3. Inicia a conversão ADC.
 	4. Aguarda a conversão ser concluída.
 	5. Lê o valor digital convertido.
 	6. Para a conversão e retorna o valor lido.

Uso: Lê sensores analógicos (joystick, potenciômetro etc.)

 -----------------------------------------------------------------------------------------*/

uint32_t ler_adc(uint32_t canal)
{
    ADC_ChannelConfTypeDef sConfig = {0};  // Estrutura de configuração do canal ADC
    sConfig.Channel = canal;               // Seleciona o canal desejado
    sConfig.Rank = 1;                      // Define a prioridade de conversão
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES; // Tempo de amostragem do ADC
    HAL_ADC_ConfigChannel(&hadc1, &sConfig); // Aplica a configuração no ADC

    HAL_ADC_Start(&hadc1);                  // Inicia a conversão
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Espera a conversão terminar (sem timeout)
    uint32_t valor = HAL_ADC_GetValue(&hadc1); // Lê o valor convertido
    HAL_ADC_Stop(&hadc1);                   // Para o ADC

    return valor;                           // Retorna o valor lido
}

/* Função set_fan_duty() ------------------------------------------------------------------

Objetivo: Define o valor do duty cycle (ciclo de trabalho) para um canal
		  PWM de um timer, limitando o valor a 99%.

Como funciona:
 	1. Garante que o valor de duty não ultrapasse 99.
 	2. Usa a macro __HAL_TIM_SET_COMPARE() para ajustar o sinal PWM no canal indicado.

Uso: Controla a velocidade de uma ventoinha ou motor com PWM.

 -----------------------------------------------------------------------------------------*/

void set_fan_duty(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t duty)
{
    if (duty > 99) duty = 99;  // Limita o duty cycle a 99%
    __HAL_TIM_SET_COMPARE(htim, channel, duty);  // Define o duty cycle no canal PWM
}


/* Função atualizar_ventoinhas() ---------------------------------------------------------

Objetivo: Atualiza a velocidade das ventoinhas com base nas
 	 	  leituras dos eixos X e Y de um joystick.

Como funciona:
 	1. Lê os valores analógicos dos canais ADC associados ao joystick.
 	2. Define uma margem de tolerância para evitar oscilações em torno do centro.
	3. Para cada eixo: Se o valor estiver acima da margem, diminui o duty
	   (e, portanto, reduz a velocidade da ventoinha). Se abaixo, aumenta o duty.
	4. Chama set_fan_duty() para aplicar a mudança.

Uso: Controla duas ventoinhas conforme a inclinação
	 do joystick (X → ventoinha 1; Y → ventoinha 2).

 -----------------------------------------------------------------------------------------*/

void atualizar_ventoinhas(uint8_t *duty_x, uint8_t *duty_y)
{
    int MARGEM = 450;  // Margem de tolerância para movimentação do joystick

    uint32_t adc_x = ler_adc(ADC_CHANNEL_6);  // Lê valor do eixo X (PA6)
    uint32_t adc_y = ler_adc(ADC_CHANNEL_9);  // Lê valor do eixo Y (PB1)

    // Controle do eixo X → PB0 (TIM3 CH3)
    if (adc_x > (2048 + MARGEM) && *duty_x > 0)
    {
        (*duty_x)--;  // Diminui o duty se o joystick for para a direita
        set_fan_duty(&htim2, TIM_CHANNEL_2, *duty_x);
        HAL_Delay(2); // Pequeno delay para estabilidade
    }
    else if (adc_x < (2048 - MARGEM) && *duty_x < 99)
    {
        (*duty_x)++;  // Aumenta o duty se o joystick for para a esquerda
        set_fan_duty(&htim2, TIM_CHANNEL_2, *duty_x);
        HAL_Delay(2);
    }

    // Controle do eixo Y → PB4 (TIM3 CH1)
    if (adc_y > (2048 + MARGEM) && *duty_y > 0)
    {
        (*duty_y)--;  // Diminui o duty se o joystick for para cima
        set_fan_duty(&htim3, TIM_CHANNEL_1, *duty_y);
        HAL_Delay(2);
    }
    else if (adc_y < (2048 - MARGEM) && *duty_y < 99)
    {
        (*duty_y)++;  // Aumenta o duty se o joystick for para baixo
        set_fan_duty(&htim3, TIM_CHANNEL_1, *duty_y);
        HAL_Delay(2);
    }
}


/* Função atualizar_display_completo() ---------------------------------------------------------

Objetivo: Atualiza todas as informações mostradas no display OLED (SSD1306).

Como funciona:
 	1. Limpa a tela (ssd1306_Fill(Black)).
 	2. Escreve:
 		2.1 Percentual de PWM das ventoinhas (linha 0);
 		2.2 Estados dos botões A e B (linha 12);
 		2.3 Valores RGB da matriz de LEDs (linha 24);
 		2.4 Índices de composição e intensidade de cor (linha 36);
 		2.5 Temperatura e umidade do sensor SHT20 (linha 48).
	3. Atualiza o display

Uso: Exibe status em tempo real dos atuadores e sensores no sistema.


 -----------------------------------------------------------------------------------------*/

static void atualizar_display_completo(uint8_t duty1,
                                       uint8_t duty2,
                                       uint8_t btnA,
                                       uint8_t btnB,
                                       uint8_t G,
                                       uint8_t R,
                                       uint8_t B)
{
    char buf[32];
    ssd1306_Fill(Black);  // Limpa a tela do display

    // Exibe porcentagens das ventoinhas (duty1 e duty2)
    uint8_t pct1 = (duty1 * 100) / 99;
    uint8_t pct2 = (duty2 * 100) / 99;
    snprintf(buf, sizeof(buf), "V1:%3d%% V2:%3d%%", pct1, pct2);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(buf, Font_7x10, White);

    // Exibe estados dos botões A e B
    snprintf(buf, sizeof(buf), "A/B:%d/%d", btnA, btnB);
    ssd1306_SetCursor(0, 12);
    ssd1306_WriteString(buf, Font_7x10, White);

    // Exibe valores RGB atuais
    snprintf(buf, sizeof(buf), "GRB:[%3d,%3d,%3d]", G, R, B);
    ssd1306_SetCursor(0, 24);
    ssd1306_WriteString(buf, Font_7x10, White);

    // Exibe índices de composição e intensidade
    snprintf(buf, sizeof(buf), "C:%u I:%u", composition_idx, intensity_idx);
    ssd1306_SetCursor(0, 36);
    ssd1306_WriteString(buf, Font_7x10, White);

    // Exibe temperatura e umidade do sensor SHT-20
    snprintf(buf, sizeof(buf),
             "S T:%2.1fC H:%2.1f%%",
             sht_temp, sht_hum);
    ssd1306_SetCursor(0, 48);
    ssd1306_WriteString(buf, Font_7x10, White);

    ssd1306_UpdateScreen();  // Atualiza o display com o novo conteúdo
}


/* Função check_buttons() ---------------------------------------------------------

Objetivo: Verifica o estado dos botões A (PB12) e B (PA15),
 	 	  aplicando debounce e atualizando índices globais conforme cliques.

Como funciona:
 	1. Lê o estado atual dos botões.
 	2. Verifica se houve mudança (para aplicar debounce).
 	3. Se o botão A foi pressionado: Incrementa o índice de intensidade e atualiza a matriz
 	4. Se o botão B foi pressionado: Incrementa o índice de composição e atualiza a matriz

Uso: Muda configurações da matriz RGB com os botões físicos.


 -----------------------------------------------------------------------------------------*/

static void check_buttons(void)
{
    uint32_t now = HAL_GetTick();  // Obtém o tempo atual do sistema

    // --- BOTÃO A ---
    GPIO_PinState rawA = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);  // Lê o estado do botão A
    if (rawA != lastRawA) {  // Detecta mudança de estado (possível bounce)
        lastDebounceTimeA = now;
        lastRawA = rawA;
    }
    if ((now - lastDebounceTimeA) > DEBOUNCE_MS) {  // Verifica se passou tempo suficiente para considerar o estado estável
        if (rawA == GPIO_PIN_RESET && !btnA_pressed) {
            btnA_pressed = 1;
            intensity_idx = (intensity_idx + 1) % 5;  // Muda índice de intensidade
            update_matrix();  // Atualiza matriz de LED
        } else if (rawA == GPIO_PIN_SET) {
            btnA_pressed = 0;  // Reseta flag de botão solto
        }
    }

    // --- BOTÃO B ---
    GPIO_PinState rawB = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);  // Lê o estado do botão B
    if (rawB != lastRawB) {
        lastDebounceTimeB = now;
        lastRawB = rawB;
    }
    if ((now - lastDebounceTimeB) > DEBOUNCE_MS) {
        if (rawB == GPIO_PIN_RESET && !btnB_pressed) {
            btnB_pressed = 1;
            composition_idx = (composition_idx + 1) % 4;  // Muda índice de composição
            update_matrix();
        } else if (rawB == GPIO_PIN_SET) {
            btnB_pressed = 0;
        }
    }
}

/* Função update_matrix() ---------------------------------------------------------

Objetivo: Atualizar a matriz de LEDs WS2812 com a cor correta e intensidade adequada

Como funciona:
 	1. Reinicializa o hardware da fita de LEDs, preparando o sistema para enviar dados novos
 	2. Calcula a intensidade da luz com base no índice atual de intensidade
 	3. Converte um valor percentual em um valor entre 0 e 255 para controle da luminosidade
 	4. Obtém a composição de cores selecionada (valores de verde, vermelho e azul)
 	5. Determina o maior valor para normalizar as componentes de cor proporcionalmente
 	   garantindo que a intensidade máxima seja mantida
 	6. Atualiza o array que representa as cores de cada LED na fita e configura
 	   todos os LEDs a mesma cor com os valores normalizados de G, R e B.


Uso: Escreve todos os pixels com a cor definida e envia os dados via SPI para a fita de LEDs,
 	 atualizando a iluminação visualmente.


 -----------------------------------------------------------------------------------------*/

static void update_matrix(void)
{
    // Inicializa o hardware da fita de LEDs WS2812 (reset + preparação)
    ws2812_init();

    // Calcula a intensidade da luz convertendo percentual em valor 0-255
    uint8_t I = (intensity_levels[intensity_idx] * 255 + 50) / 100;

    // Pega o ponteiro para o array da composição atual de cores (G, R, B)
    const uint8_t *comp = compositions[composition_idx];

    // Determina o maior valor da composição para normalizar as cores
    uint8_t max_pct = comp[0];
    if (comp[1] > max_pct) max_pct = comp[1];
    if (comp[2] > max_pct) max_pct = comp[2];

    // Normaliza a cor verde proporcionalmente à intensidade e ao maior valor
    uint8_t G = (uint32_t)comp[0] * I / max_pct;
    // Normaliza a cor vermelha proporcionalmente à intensidade e ao maior valor
    uint8_t R = (uint32_t)comp[1] * I / max_pct;
    // Normaliza a cor azul proporcionalmente à intensidade e ao maior valor
    uint8_t B = (uint32_t)comp[2] * I / max_pct;

    // Atualiza todos os LEDs no array de cor para a composição calculada
    for (int i = 0; i < NUM_LEDS; i++) {
        ws2812_rgb[i][0] = G; // verde
        ws2812_rgb[i][1] = R; // vermelho
        ws2812_rgb[i][2] = B; // azul
    }

    // Envia a cor para todos os pixels da fita via SPI
    ws2812_pixel_all(R, G, B);

    // Dispara o envio dos dados para os LEDs
    ws2812_send_spi();
}

/* Função check_switch() ---------------------------------------------------------

Objetivo: Implementa o debouncing do botão conectado ao pino PA3, para evitar
 	 	  leituras falsas devido ao ruído elétrico causado pelo contato mecânico do botão.
 	 	  Além disso ela troca de página do display


Como funciona:
 	1. Captura o tempo atual em milissegundos e lê o estado do pino do botão
 	2. Compara com o último estado lido para detectar mudanças
 	3. Caso haja uma mudança no estado do botão, ela atualiza o tempo do
 	   último evento de mudança para controlar o atraso necessário para o debouncing
 	4. Se o tempo decorrido desde a última mudança ultrapassa o intervalo de debounce definido:
 		4.1 Verifica se o botão foi pressionado ou solto
 		4.2 Alterna a página do display quando o botão é detectado como pressionado.

Uso: Controla uma flag que indica se o botão está pressionado
     para evitar múltiplas alternâncias da tela causadas por leituras repetidas.


 -----------------------------------------------------------------------------------------*/

static void check_switch(void)
{
    // Obtém o tempo atual em ms do sistema
    uint32_t now = HAL_GetTick();

    // Lê o estado atual do pino do botão
    GPIO_PinState raw = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);

    // Detecta mudança no estado do botão
    if (raw != lastRawSw) {
        // Atualiza o tempo da última mudança para controle de debounce
        lastDebounceTimeSw = now;
        // Atualiza o último estado bruto lido
        lastRawSw = raw;
    }

    // Verifica se passou o tempo de debounce desde a última mudança
    if ((now - lastDebounceTimeSw) > DEBOUNCE_MS) {
        // Se botão foi pressionado (nível baixo) e antes não estava pressionado
        if (raw == GPIO_PIN_RESET && !sw_pressed) {
            // Marca botão como pressionado
            sw_pressed = 1;
            // Alterna a página do display (de 0 para 1 ou vice-versa)
            display_page = 1 - display_page;
        }
        // Se botão está solto (nível alto), reseta flag de pressionado
        else if (raw == GPIO_PIN_SET) {
            sw_pressed = 0;
        }
    }
}

/* Função update_offsets() ---------------------------------------------------------

Objetivo: Ajusta os offsets de temperatura e umidade com base no movimento do joystick

Como funciona:
 	1. Define os valores médios e zona morta (deadzone) para os eixos X e Y do joystick,
 	   para evitar ajustes constantes com pequenas variações
 	2. No eixo Y, referente ao ajuste da temperatura:
 	 	2.1 Lê o valor ADC e, se estiver acima ou abaixo da zona morta, incrementa
 	 	  	ou decrementa o offset da temperatura,  respeitando limites mínimos e
 	 	  	máximos para evitar valores fora da faixa
 	 	2.2 Calcula a temperatura final ajustada somando o offset à leitura do sensor,
 	 	    limitando seu valor entre 15°C e 40°C, para evitar leituras inválidas.
 	3. No eixo X, referente ao ajuste da umidade:
 		3.1  Lê o valor ADC, ajusta o offset dentro de limites, e calcula a umidade final
 		3.2  A umidade é limitada para ficar entre um duty cycle de 0% e 100%


Uso: Mapeia a temperatura e umidade final para um valor de duty cycle entre 0 e 100
 	 para controlar a velocidade da ventoinha associada ao canal PWM.


 -----------------------------------------------------------------------------------------*/

static void update_offsets(void)
{
    const int MID      = 2048;  // Valor central do ADC para joystick (12 bits)
    const int DEADZONE = 300;   // Zona morta para evitar pequenos ajustes

    // --- Temperatura (eixo Y - ADC_CHANNEL_9) ---
    // Lê valor do ADC no eixo Y do joystick
    uint32_t ay = ler_adc(ADC_CHANNEL_9);

    // Se inclinação para cima (acima da zona morta) e offset maior que limite inferior
    if (ay > (MID + DEADZONE) && temp_offset > -25) {
        temp_offset--; // Decrementa offset temperatura
    }
    // Se inclinação para baixo (abaixo da zona morta) e offset menor que limite superior
    else if (ay < (MID - DEADZONE) && temp_offset < +25) {
        temp_offset++; // Incrementa offset temperatura
    }

    // Garante que o offset fique dentro do intervalo permitido [-25, +25]
    if (temp_offset < -25) temp_offset = -25;
    if (temp_offset > +25) temp_offset = +25;

    // Calcula temperatura final somando o offset à leitura do sensor
    float temp_meas = sht_temp + temp_offset;

    // Limita a temperatura final para o intervalo [15°C, 40°C]
    if (temp_meas < 15.0f) temp_meas = 15.0f;
    if (temp_meas > 40.0f) temp_meas = 40.0f;

    // Mapeia temperatura [15..40] para duty cycle PWM [0..100]
    uint8_t duty_temp = (uint8_t)((temp_meas - 15.0f) * 100.0f / 25.0f);

    // Ajusta duty cycle da ventoinha associada ao canal 1 do temporizador 3
    set_fan_duty(&htim3, TIM_CHANNEL_1, duty_temp);

    // --- Umidade (eixo X - ADC_CHANNEL_6) ---
    // Lê valor do ADC no eixo X do joystick
    uint32_t ax = ler_adc(ADC_CHANNEL_6);

    // Se inclinação para a direita e offset maior que limite inferior
    if (ax > (MID + DEADZONE) && hum_offset > -50) {
        hum_offset--; // Decrementa offset umidade
    }
    // Se inclinação para a esquerda e offset menor que limite superior
    else if (ax < (MID - DEADZONE) && hum_offset < +50) {
        hum_offset++; // Incrementa offset umidade
    }

    // Limita o offset de umidade dentro de [-50, +50]
    if (hum_offset < -50) hum_offset = -50;
    if (hum_offset > +50) hum_offset = +50;

    // Calcula umidade final somando o offset à leitura do sensor
    float hum_meas = sht_hum + hum_offset;

    // Limita umidade final para intervalo válido [0%, 100%]
    if (hum_meas <  0.0f) hum_meas =  0.0f;
    if (hum_meas >100.0f) hum_meas =100.0f;

    // Converte umidade para duty cycle PWM diretamente (0..100)
    uint8_t duty_hum = (uint8_t)(hum_meas);

    // Ajusta duty cycle da ventoinha associada ao canal 2 do temporizador 2
    set_fan_duty(&htim2, TIM_CHANNEL_2, duty_hum);
}



/* Função display_home() ---------------------------------------------------------

Objetivo: Desenhar na tela OLED e exibe valores de temperatura, umidade, intensidade e composição da luz.


Como funciona:
 	1. Limpa a tela e prepara o buffer para montagem das strings de texto.
 	2. Calcula a temperatura final aplicando o offset e garantindo que
 	   esteja no intervalo aceitável de 15°C a 40°C
 	3. Exibe a temperatura no formato “sensor + offset = final” na primeira linha da tela
 	4. De forma semelhante, calcula e exibe a umidade final com o offset aplicado,
 	   garantindo o intervalo de 0% a 100%, na linha seguinte
 	5. Exibe a intensidade da luz, com a descrição textual conforme o índice selecionado.
 	6. Exibe a composição da cor atual da luz também baseada no índice.

Uso: Atualiza a tela OLED para mostrar todos os dados formatados e organizados.


 -----------------------------------------------------------------------------------------*/

static void display_home(void)
{
    char buf[32];                     // Buffer para armazenar as strings que serão exibidas no display
    ssd1306_Fill(Black);             // Limpa a tela do display preenchendo com a cor preta

    // --- Temperatura ---
    float temp_final = sht_temp + temp_offset; // Aplica o offset de calibração na temperatura medida
    if (temp_final < 15.0f) temp_final = 15.0f; // Limita o valor mínimo a 15 °C
    if (temp_final > 40.0f) temp_final = 40.0f; // Limita o valor máximo a 40 °C
    snprintf(buf, sizeof(buf),               // Formata a string: sensor + offset = resultado final
             "Tmp:%2.1f%+d=%2.1fC",
             sht_temp,
             temp_offset,
             temp_final);
    ssd1306_SetCursor(0,  0);                // Define a posição para exibir a string (linha 0)
    ssd1306_WriteString(buf, Font_7x10, White); // Escreve a string com fonte 7x10 na cor branca

    // --- Umidade ---
    float hum_final = sht_hum + hum_offset; // Aplica o offset de calibração na umidade medida
    if (hum_final <  0.0f) hum_final =  0.0f; // Limita o valor mínimo a 0%
    if (hum_final >100.0f) hum_final =100.0f; // Limita o valor máximo a 100%
    snprintf(buf, sizeof(buf),               // Formata a string: sensor + offset = resultado final
             "Umd:%2.1f%+d=%2.1f%%",
             sht_hum,
             hum_offset,
             hum_final);
    ssd1306_SetCursor(0, 12);                // Define a posição para exibir a string (linha 12)
    ssd1306_WriteString(buf, Font_7x10, White); // Escreve a string no display

    // --- Intensidade da luz ---
    snprintf(buf, sizeof(buf), "Intens:%s",     // Pega o rótulo correspondente ao índice atual de intensidade
             intensity_labels[intensity_idx]);
    ssd1306_SetCursor(0, 24);                   // Linha 24
    ssd1306_WriteString(buf, Font_7x10, White); // Exibe a intensidade

    // --- Composição do espectro ---
    snprintf(buf, sizeof(buf), "Comp:%s",       // Pega o rótulo correspondente ao índice atual de composição
             composition_labels[composition_idx]);
    ssd1306_SetCursor(0, 36);                   // Linha 36
    ssd1306_WriteString(buf, Font_7x10, White); // Exibe a composição do espectro

    ssd1306_UpdateScreen();                     // Atualiza o display com o conteúdo do buffer
}

/* Função SHT20_ReadTemperature() ---------------------------------------------------------

Objetivo: Realiza a leitura da temperatura do sensor SHT20 via protocolo I2C


Como funciona:
 	1. Envia o comando de medição de temperatura no modo “no-hold” ao sensor,
 	   utilizando o endereço I2C e comando apropriado
 	2. Aguarda o tempo necessário para a conversão da medida (~85 ms)
 	3. Recebe os dados da temperatura bruta enviados pelo sensor em três bytes
 	4. Calcula a temperatura real usando a fórmula fornecida no datasheet do sensor
 		4.1 Verifica se o botão foi pressionado ou solto
 		4.2 Alterna a página do display quando o botão é detectado como pressionado.

Uso: Coleta o dado enviado pelo sensor e retorna o valor da temperatura em ponto flutuante


 -----------------------------------------------------------------------------------------*/

static float SHT20_ReadTemperature(void)
{
    uint8_t cmd = 0xF3; // Comando para iniciar leitura de temperatura no modo "no hold"
    HAL_I2C_Master_Transmit(&hi2c1, 0x40<<1, &cmd, 1, HAL_MAX_DELAY); // Envia comando ao sensor no endereço 0x40
    HAL_Delay(85);       // Aguarda tempo de conversão (~85 ms, conforme datasheet do SHT20)

    uint8_t data[3];     // Buffer para receber os 2 bytes de dados + 1 byte de checksum (opcional)
    HAL_I2C_Master_Receive(&hi2c1, 0x40<<1, data, 3, HAL_MAX_DELAY); // Lê os dados do sensor via I2C

    uint16_t raw = (data[0]<<8) | data[1]; // Junta os dois primeiros bytes em um valor bruto de 16 bits

    // Converte o valor bruto para temperatura em Celsius, conforme fórmula do datasheet:
    return -46.85f + 175.72f * raw / 65536.0f;
}

/* Função SHT20_ReadHumidity() ---------------------------------------------------------

Objetivo: Realiza a leitura da umidade relativa do sensor SHT20 via protocolo I2C.


Como funciona:
 	1. Envia o comando de medição de umidade no modo “no-hold” ao sensor,
 	   usando o endereço I2C e comando adequado.
 	2. Aguarda o tempo necessário para a conversão da medida (~29 ms)
 	3. Recebe os dados brutos de umidade em três bytes do sensor,
 	   que transforma o valor bruto em porcentagem de umidade
 	4. Calcula a umidade relativa em porcentagem usando a fórmula oficial do datasheet

Uso: Coleta o dado enviado pelo sensor e retorna o valor da umidade relativa como um float..


 -----------------------------------------------------------------------------------------*/

static float SHT20_ReadHumidity(void)
{
    uint8_t cmd = 0xF5; // Comando para iniciar leitura de umidade no modo "no hold"
    HAL_I2C_Master_Transmit(&hi2c1, 0x40<<1, &cmd, 1, HAL_MAX_DELAY); // Envia comando ao sensor

    HAL_Delay(29);      // Aguarda tempo de conversão (~29 ms para umidade)

    uint8_t data[3];    // Buffer para os dados recebidos
    HAL_I2C_Master_Receive(&hi2c1, 0x40<<1, data, 3, HAL_MAX_DELAY); // Lê os dados via I2C

    uint16_t raw = (data[0]<<8) | data[1]; // Concatena os dois bytes principais da leitura

    // Converte para umidade relativa (%) conforme fórmula do datasheet:
    return -6.0f + 125.0f * raw / 65536.0f;
}


* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  // Habilita o contador de ciclos do DWT (Data Watchpoint and Trace) — útil para criar atrasos com precisão sub-microsegundos
  CoreDebug->DEMCR   |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL         |= DWT_CTRL_CYCCNTENA_Msk;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  // Reset de todos os periféricos, inicializa a interface Flash e o Systick (temporizador do sistema)
  HAL_Init();

  /* USER CODE BEGIN Init */
  // Espaço reservado para inicializações personalizadas antes da configuração do sistema
  /* USER CODE END Init */

  // Configuração do clock do sistema
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  // Espaço reservado para configurações do sistema antes da inicialização dos periféricos
  /* USER CODE END SysInit */

  // Inicializa todos os periféricos configurados no CubeMX
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_SPI5_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  // Inicia geração de sinal PWM para controle das ventoinhas nas portas PB3 (X) e PB4 (Y)
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // Ventoinha no eixo Y
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  // Ventoinha no eixo X

  // Inicializa o display OLED SSD1306
  ssd1306_Init();

  // Inicializa os LEDs WS2812 (RGB), limpando o buffer e enviando sinal de reset
  ws2812_init();

  // Atualiza a matriz de LEDs com cor e intensidade iniciais
  update_matrix();

  // Define o duty cycle inicial das ventoinhas para 50% (metade da velocidade máxima)
  uint8_t duty_x = 50;
  uint8_t duty_y = 50;

  // Aplica o duty cycle nas ventoinhas
  set_fan_duty(&htim2, TIM_CHANNEL_2, duty_x); // Ventoinha X
  set_fan_duty(&htim3, TIM_CHANNEL_1, duty_y); // Ventoinha Y

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    // Atualiza a velocidade das ventoinhas com base no joystick
	    atualizar_ventoinhas(&duty_x, &duty_y);

	    // Verifica os botões A e B para mudar cor/intensidade dos LEDs
	    check_buttons();

	    // Verifica o interruptor (PA3) para trocar entre as telas de exibição
	    check_switch();

	    if (display_page == 0) {
	        // Atualiza os offsets de temperatura e umidade
	        update_offsets();

	        // Lê os valores de temperatura e umidade do sensor SHT20
	        sht_temp = SHT20_ReadTemperature();
	        sht_hum  = SHT20_ReadHumidity();

	        // Mostra os dados principais na tela "home"
	        display_home();
	    }
	    else {
	    	// Atualiza novamente as ventoinhas (controle ativo apenas na tela de debug)
	    	atualizar_ventoinhas(&duty_x, &duty_y);

	        // Atualiza a tela de debug com dados detalhados
	        atualizar_display_completo(
	            duty_x,             // Duty cycle da ventoinha X
	            duty_y,             // Duty cycle da ventoinha Y
	            btnA_pressed,       // Estado do botão A
	            btnB_pressed,       // Estado do botão B
	            ws2812_rgb[0][0],   // Valor do canal R do LED
	            ws2812_rgb[0][1],   // Valor do canal G do LED
	            ws2812_rgb[0][2]    // Valor do canal B do LED
	        );
	    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

* @brief Configuração do Clock do Sistema
* @retval Nenhum
*/
void SystemClock_Config(void)
{
// Estruturas para configurar os osciladores e os clocks do sistema
RCC_OscInitTypeDef RCC_OscInitStruct = {0};
RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

// Habilita o clock do controlador de energia
__HAL_RCC_PWR_CLK_ENABLE();

// Configura a escala de voltagem do regulador interno
__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

// Inicialização do oscilador:
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
RCC_OscInitStruct.HSEState = RCC_HSE_ON;
RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
RCC_OscInitStruct.PLL.PLLM = 25;
RCC_OscInitStruct.PLL.PLLN = 192;
RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
RCC_OscInitStruct.PLL.PLLQ = 4;

// Aplica a configuração do oscilador
if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
{
  // Se ocorrer erro, chama função de tratamento
  Error_Handler();
}

// Configuração dos clocks para os barramentos AHB, APB1 e APB2:
RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2; // Define quais clocks serão configurados
RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

// Aplica a configuração dos clocks
if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
{
  // Se ocorrer erro, chama função de tratamento
  Error_Handler();
}
}

/**
  * @brief Função de Inicialização do ADC1
  * @param Nenhum
  * @retval Nenhum
  */

static void MX_ADC1_Init(void)
{
	  /* USER CODE BEGIN ADC1_Init 0 */

	  /* USER CODE END ADC1_Init 0 */

	  ADC_ChannelConfTypeDef sConfig = {0};

	  /* USER CODE BEGIN ADC1_Init 1 */

	  /* USER CODE END ADC1_Init 1 */

  /* Configura as características gerais do ADC1 */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler(); // Trata erro na inicialização do ADC
  }

  /* Configura o canal do ADC a ser usado */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler(); // Trata erro de configuração do canal
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */


static void MX_I2C1_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */

  // Define que usará a interface I2C1
  hi2c1.Instance = I2C1;

  // Configura a velocidade de clock I2C (100 kHz — modo padrão)
  hi2c1.Init.ClockSpeed = 100000;

  // Define a razão cíclica duty cycle (aplicável em modo rápido — aqui fixado em 2)
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;

  // Endereço primário do dispositivo (usado apenas em modo escravo)
  hi2c1.Init.OwnAddress1 = 0;

  // Usa endereçamento de 7 bits (comum na maioria dos dispositivos I2C)
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;

  // Desativa o uso de segundo endereço escravo (modo dual)
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;

  // Segundo endereço (não usado pois modo dual está desabilitado)
  hi2c1.Init.OwnAddress2 = 0;

  // Desativa o modo "general call" (endereçamento coletivo de todos os escravos)
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;

  // Permite o "clock stretching" (esticar o clock quando o escravo estiver ocupado)
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  // Inicializa o periférico I2C1 com as configurações acima
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    // Se falhar na inicialização, chama o manipulador de erro
    Error_Handler();
  }

  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}


/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */

static void MX_I2C2_Init(void)
{
  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */

  // Define que usará a interface I2C2
  hi2c2.Instance = I2C2;

  // Configura a velocidade de clock I2C (100 kHz — modo padrão)
  hi2c2.Init.ClockSpeed = 100000;

  // Define a razão cíclica duty cycle (aplicável em modo rápido — aqui fixado em 2)
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;

  // Endereço primário do dispositivo (usado apenas em modo escravo)
  hi2c2.Init.OwnAddress1 = 0;

  // Usa endereçamento de 7 bits
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;

  // Desativa o uso de segundo endereço escravo (modo dual)
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;

  // Segundo endereço (não usado pois modo dual está desabilitado)
  hi2c2.Init.OwnAddress2 = 0;

  // Desativa o modo "general call"
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;

  // Permite "clock stretching"
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  // Inicializa o periférico I2C2 com as configurações acima
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    // Se falhar na inicialização, chama o manipulador de erro
    Error_Handler();
  }

  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{
  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */

  /* Configuração dos parâmetros do SPI5 */
  hspi5.Instance = SPI5;                        // Seleciona a instância SPI5 do hardware
  hspi5.Init.Mode = SPI_MODE_MASTER;           // Define o SPI como mestre
  hspi5.Init.Direction = SPI_DIRECTION_2LINES; // Comunicação full-duplex com 2 linhas
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;     // Tamanho dos dados: 8 bits
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;   // Polaridade do clock: nível baixo quando o clock está inativo
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;       // Fase do clock: captura na primeira borda
  hspi5.Init.NSS = SPI_NSS_SOFT;                // NSS gerenciado por software
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; // Prescaler do clock do SPI (fator de divisão)
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;      // Primeiro bit enviado: bit mais significativo
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;      // Modo TI desabilitado (modo padrão SPI)
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE; // CRC desabilitado
  hspi5.Init.CRCPolynomial = 10;                // Polinômio CRC (não usado)

  /* Inicializa o SPI5 com os parâmetros configurados */
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    /* Se falhar, chama a rotina de tratamento de erro */
    Error_Handler();
  }

  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{
  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};  // Configuração da fonte do clock do timer
  TIM_MasterConfigTypeDef sMasterConfig = {0};      // Configuração mestre/escravo do timer
  TIM_OC_InitTypeDef sConfigOC = {0};               // Configuração do modo PWM no canal do timer

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */

  /* Configuração da instância TIM2 e parâmetros básicos */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 4799;                      // Prescaler para dividir o clock (ex: se clk=48MHz, timer = 48MHz/(4799+1) = 10kHz)
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;                            // Valor de auto-reload (periodo do timer = 100 ciclos do timer)
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;// Sem divisão do clock do timer
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // Atualização imediata do valor do período

  /* Inicializa a base do timer com os parâmetros configurados */
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();  // Tratamento de erro se falhar a inicialização do timer base
  }

  /* Configura a fonte do clock para o timer como clock interno */
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();  // Tratamento de erro se falhar a configuração do clock
  }

  /* Inicializa o timer para modo PWM */
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();  // Tratamento de erro se falhar a inicialização do PWM
  }

  /* Configurações do modo mestre do timer (desabilitado neste caso) */
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();  // Tratamento de erro se falhar a sincronização mestre
  }

  /* Configura o canal 2 para modo PWM */
  sConfigOC.OCMode = TIM_OCMODE_PWM1;           // Modo PWM tipo 1 (nível alto até o pulso)
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();  // Tratamento de erro se falhar a configuração do canal PWM
  }

  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

  /* Configura os GPIOs relacionados ao timer para modo alternativo PWM */
  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */

static void MX_TIM3_Init(void)
{
  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  //Estruturas de configuração
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */

  htim3.Instance = TIM3;                              // Seleciona o timer 3 para configuração
  htim3.Init.Prescaler = 4799;                        // Prescaler: divide a frequência do clock do timer por 4800 (4799 + 1)
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;        // Contagem crescente (de 0 até o valor do período)
  htim3.Init.Period = 99;                             // Valor máximo de contagem antes de reiniciar (timer period)
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // Sem divisão do clock do timer
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; // Atualização imediata do valor do auto-reload

  // Inicializa a base do timer com as configurações acima
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();  // Tratamento de erro caso a inicialização falhe
  }

  // Configura o clock do timer para usar a fonte interna
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();  // Tratamento de erro caso a configuração falhe
  }

  // Inicializa o timer para funcionar em modo PWM
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();  // Tratamento de erro caso a inicialização falhe
  }

  // Configurações para sincronização do timer (modo mestre)
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;  // Nenhum trigger especial de saída
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; // Modo mestre/escravo desabilitado

  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();  // Tratamento de erro caso a configuração falhe
  }

  // Configuração do canal PWM
  sConfigOC.OCMode = TIM_OCMODE_PWM1;             // Modo PWM tipo 1
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  // Configura o canal 1 do timer com a configuração PWM acima
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();  // Tratamento de erro caso a configuração falhe
  }

  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);                    // Inicialização adicional para o timer (GPIO e outros)
  /* USER CODE END TIM3_Init 2 */

  HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();  // Ativa o clock do GPIO porta H
  __HAL_RCC_GPIOA_CLK_ENABLE();  // Ativa o clock do GPIO porta A
  __HAL_RCC_GPIOB_CLK_ENABLE();  // Ativa o clock do GPIO porta B

  /*Configure PA3 (joystick switch) */
  GPIO_InitStruct.Pin  = GPIO_PIN_3;              // Seleciona o pino 3 da porta A
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;         // Configura como entrada digital
  GPIO_InitStruct.Pull = GPIO_PULLUP;             // Habilita resistor pull-up interno
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);         // Inicializa o GPIOA pin 3 com essas configurações

  /*Configure PB12 (botão A) */
  GPIO_InitStruct.Pin  = GPIO_PIN_12;             // Seleciona o pino 12 da porta B
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;         // Configura como entrada digital
  GPIO_InitStruct.Pull = GPIO_PULLUP;             // Habilita resistor pull-up interno
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);         // Inicializa o GPIOB pin 12 com essas configurações

  /*Configure PA15 (botão B) */
  GPIO_InitStruct.Pin  = GPIO_PIN_15;             // Seleciona o pino 15 da porta A
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;         // Configura como entrada digital
  GPIO_InitStruct.Pull = GPIO_PULLUP;             // Habilita resistor pull-up interno
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);         // Inicializa o GPIOA pin 15 com essas configurações

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // --- Configuração dos pinos para I2C1 (PB6 = SCL, PB7 = SDA) ---
  __HAL_RCC_GPIOB_CLK_ENABLE();                    // Garante que o clock do GPIOB está ativo
  GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;  // Seleciona pinos 6 e 7 da porta B
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;    // Modo função alternativa Open-Drain (para I2C)
  GPIO_InitStruct.Pull      = GPIO_PULLUP;        // Resistor pull-up interno habilitado (necessário para I2C)
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH; // Alta velocidade para sinais I2C
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;      // Função alternativa 4 para I2C1
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);         // Inicializa GPIOB pinos 6 e 7 para I2C1
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
