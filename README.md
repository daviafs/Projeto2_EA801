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
