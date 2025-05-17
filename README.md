# Projeto2_EA801

Projeto para a disciplina EA801- Laboratório Projetos Sistemas Embarcados (FEEC-UNICAMP)

Desenvolvido por: Davi A. F. de Souza & Gabriel M. de Andrade

Docente responsável: Antônio A. F. Quevedo

Data: 16 de Maio, 2025

## Descrição:
Título: SISTEMA DE CONTROLE PWM PARA ILUMINAÇÃO LED E CLIMATIZAÇÃO EM AGRICULTURA INDOOR

Linguagem: C

Plataforma: SMT32CubeIDE

MCU: STM32F411 BlackPill (Através da placa BitDogLab)

Bibliotecas utilizadas: arquivos '.c' e '.h' disponíveis na pasta "All_Files.zip"

Diretório completo da SM32CubeIDE: disponíveis na pasta "teste.zip"
	
## Objetivo do Projeto

O projeto consiste em desenvolver um software de controle via PWM para aplicações de iluminação LED e climatização  baseado em ventoinhas, garantindo o controle de temperatura e umidade relativa em estufas, por exemplo,. A proposta é simular as condições de um ambiente controlado para plantas, com soluções que permitem ajustes e monitoramento em tempo real. Para isso, o projeto conta com a placa BitDogLab e microcontrolador STM32F411 BlackPill para garantir as seguintes funcionalidades:

1. Ajuste do espectro de luz e sua intensidade com base na definição de parâmetros desejados pelo usuário, controlado através dos botões na placa;

2. Emulação de um sensor através do joystick para fornecer a informação analógica das variáveis simulando a temperatura e umidade relativa; 

3. Ajuste da velocidade da ventoinha para ventilação e exaustão a partir das mudanças nas variáveis de temperatura e umidade relativa;

4. Apresentação dos valores das variáveis emuladas e status do sistema via display.

Para alcançar os objetivos propostos, foi desenvolvido um programa em C, a nível de registradores, via software STM32CubeIDE. O upload do código na placa necessitou de um dispositivo ST-Link para conexão via porta USB. A lógica do algotitmo a ser desenvolvido contou, de maneira geral, com as seguintes funções:

Controle PWM da ventoinhas:
- Envio e leitura de um sinal ADC de 8 bits para o módulo PWM
- Setar o Duty Cycle para cada ventoina ventoinhas
- Atualizar as ventoinhas: offset e regulação pelo joystick

Comunicação I2C com sensor:
- Leitura de temperatura
- Leitura de umidade relativa

Atualização do sistema:
- Atualiza o que está sendo apresentado no display OLED
- Checar o estado dos botões (leitura, debounce, checar memória)
- Atualiza os valores da Matriz LED GRB
Atualiza valo do offset de tempemperatura e umidade relativa


## Mapeamento das portas (teste.ioc) ![MapeamentoPortas](https://github.com/user-attachments/assets/f4c24dea-967b-4e9c-9ea4-fa6a582366e7)

## Fluxograma![Fluxograma_projeto2](https://github.com/user-attachments/assets/152b5a93-73d6-4521-8fc4-7513ee5543ed)

## Imagens de funcionamento

![Funcionamento1](https://github.com/user-attachments/assets/41a8247f-cd21-41ba-a354-7393d1e4cb7c)
Descrição da imagem: Do lado esquerdo, o modo de operação considerando a composição de cor ‘Full espectro’ e 35% de intensidade (Médio), e valores de temperatura e umidade relativa. Do lado direito,  o modo de operação considerando a composição de cor ‘Pink’ e 10% de intensidade. Ambos apresentam os valores para temperatura e umidade relativa. No loop principal, o sistema atualiza continuamente a velocidade das ventoinhas com base na leitura do joystick, verifica os botões A e B para ajuste da intensidade e cor dos LEDs, e monitora o estado de um switch que alterna entre duas páginas de exibição: Home e Debug.

![Funcionamento2](https://github.com/user-attachments/assets/33ab90f5-a6bf-406e-a859-47a422eb76d0)

Descrição da imagem: Diferenças entres as informações apresentadas pelo display OLED nos modos ‘Home’, à esquerda (mais “user-friendly”), e ‘Debug’ (suporte à falha), à direita. Diferentemente da tela principal, a tela de debug oferece uma interface completa para monitoramento e análise detalhada do funcionamento do sistema em tempo real. Nessa tela, o usuário pode visualizar os valores atualizados da velocidade de cada ventoinha (duty cycle), os dados reais de temperatura e umidade lidos pelo sensor SHT20, o estado dos botões A e B (pressionados ou não), bem como os sinais enviados à matriz de LEDs RGB, permitindo observar o controle preciso da intensidade luminosa. Além disso, a interface exibe o tipo de composição de cor selecionado e sua intensidade atual, facilitando o acompanhamento do comportamento visual dos LEDs


## Especificações dos periféricos

Sensor de Umidade e Temperatura
- Modelo: SHT20
- Encapsulamento: IP65
- Medição  (UR): 0 a 100 % ± 3%
- Medição (Temp.): -40°C a 125°C ± 0,3°C 
- Tensão: 3 a 5,5V DC;
- Interface de comunicação: I2C;
- Tempo de resposta: 8s;
- Função: Realizar medições temporizadas de temperatura e umidade relativa do ambiente e envio dos dados para a STM32 black pill

Módulo Driver PWM 
- Modelo: D4184;
- Tensão: 5-36 VDC;
- Tensão de PWM: 3,3-20V;
- Frequência do PWM = 0-20 KHZ
- Corrente: 15A;
- Potência: 400W;
- Função: Controle da velocidade das ventoinhas através dos sinais da black pill

Ventoinhas
- Motor Brushless CC
- Tensão de Entrada: 12V
- Corrente: 0,18 A
- Potência: 2,16 W
- Função: Uma ventoinha foi usada para controle de temperatura (ventilação) e outra para controle de umidade (exaustão)

Conversor AC-DC:
- Modelo: ADP-30BW K
- Tensão de Entrada: 100-240 VAC (60Hz)
- Corrente de Entrada: 1 A (Alternada)
- Tensão de Saída: 12 VDC
- Corrente de Saída: 2,5 A (Contínua)
- Função: Alimentação das ventoinhas



