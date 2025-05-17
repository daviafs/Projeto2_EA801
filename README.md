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


## Mapeamento das portas (teste.ioc) ![MapeamentoPortas](https://github.com/user-attachments/assets/f4c24dea-967b-4e9c-9ea4-fa6a582366e7)

## Fluxograma
![Projeto1 drawio](https://github.com/user-attachments/assets/6de09160-3b30-433d-8074-248ff48ea927)

## Imagens de funcionamento

![Funcionamento1](https://github.com/user-attachments/assets/41a8247f-cd21-41ba-a354-7393d1e4cb7c)
Descrição da imagem: Do lado esquerdo, o modo de operação considerando a composição de cor ‘Full espectro’ e 35% de intensidade (Médio), e valores de temperatura e umidade relativa. Do lado direito,  o modo de operação considerando a composição de cor ‘Pink’ e 10% de intensidade. Ambos apresentam os valores para temperatura e umidade relativa. No loop principal, o sistema atualiza continuamente a velocidade das ventoinhas com base na leitura do joystick, verifica os botões A e B para ajuste da intensidade e cor dos LEDs, e monitora o estado de um switch que alterna entre duas páginas de exibição: Home e Debug.

![Funcionamento2](https://github.com/user-attachments/assets/33ab90f5-a6bf-406e-a859-47a422eb76d0)

Descrição da imagem: Diferenças entres as informações apresentadas pelo display OLED nos modos ‘Home’, à esquerda (mais “user-friendly”), e ‘Debug’ (suporte à falha), à direita. Diferentemente da tela principal, a tela de debug oferece uma interface completa para monitoramento e análise detalhada do funcionamento do sistema em tempo real. Nessa tela, o usuário pode visualizar os valores atualizados da velocidade de cada ventoinha (duty cycle), os dados reais de temperatura e umidade lidos pelo sensor SHT20, o estado dos botões A e B (pressionados ou não), bem como os sinais enviados à matriz de LEDs RGB, permitindo observar o controle preciso da intensidade luminosa. Além disso, a interface exibe o tipo de composição de cor selecionado e sua intensidade atual, facilitando o acompanhamento do comportamento visual dos LEDs
