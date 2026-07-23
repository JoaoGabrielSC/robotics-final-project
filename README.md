# Formação virtual LIMO–Bebop 2

Projeto da disciplina **Robótica Móvel — UFES (2026/1)**. Controle de formação por estrutura
virtual entre um robô terrestre diferencial (AgileX LIMO) e um quadrimotor (Parrot Bebop 2), com
desvio de obstáculo em espaço nulo, testado em voo real na arena do LAB-AIR.

> O Bebop 2 é a plataforma aérea adotada neste projeto.

## Script principal

O controlador usado no ensaio final é **`matlab/limo_bebop_final.m`** (mesma lógica de
`final_project/limo_bebop.m`, que é a versão efetivamente submetida e reportada). Ele:

1. controla o ponto de interesse do LIMO (deslocado `a = 0,10 m` à frente do centro do robô) por
   um laço cinemático saturado + compensador dinâmico;
2. mantém o Bebop num offset fixo `[ρ_f, α_f, β_f]` em relação a esse ponto, com laço externo
   `tanh`, controle cinemático de guinada e compensador dinâmico próprio;
3. dá prioridade máxima ao desvio de obstáculo, via projeção em espaço nulo de um campo
   potencial repulsivo;
4. tem decolagem e pouso comandados pelo joystick (Botão A / Botão B), com sobrescrita manual
   contínua pelos eixos analógicos e uma janela de afastamento manual antes do pouso automático.

Os scripts `matlab/formacao_2.m` (versão histórica, mais antiga) e os testes isolados
(`matlab/teste_*.m`) documentam etapas intermediárias de validação, mas não são o que foi usado
no ensaio de voo relatado.

## Geometria e referência

Com `TRAJ = 1`, a referência do LIMO é uma lemniscata:

$$
x_d = 0.75\sin\left(\frac{2\pi t}{40}\right), \qquad
y_d = 0.75\sin\left(\frac{4\pi t}{40}\right)
$$

O Bebop mantém um deslocamento fixo em relação ao ponto de controle do LIMO, definido por
`ρ_f = 1,5 m`, `α_f = 0°` e `β_f = 75°` (o enunciado pede `90°`, mas essa configuração é singular
— o azimute perde efeito com o drone exatamente acima do LIMO — e um ensaio preliminar com
`β_f = 60°` levou o drone para fora da área segura; `75°` foi o valor efetivamente validado em
voo). O obstáculo é um cilindro centrado em `[-0.20; 0.425] m`, com raio físico de `0.15 m` e
zona de influência de `0.25 m`. O controlador roda a `1/30 s` por ciclo.

## Estrutura do repositório

```text
matlab/
  limo_bebop_final.m               Controlador final (LIMO + Bebop), em seções independentes
  limo_bebop.m                     Versão anterior, sem a divisão em seções
  formacao_2.m                     Versão histórica (espaço de cluster)
  teste_*.m                        Testes isolados (convergência, joystick, lemniscata do LIMO)
  simulador_formacao_2.m           Simulação sem hardware
sim/
  visualizar_auditoria_formacao.py Visualizador de auditoria (TXT -> gráficos/animação)
docs/
  equacoes_controle.md             Equações do controlador, com referência ao livro-texto
  diagrama_loop_controle.md        Diagrama (mermaid) do laço de controle
  variaveis_formacao_2.md          Glossário de variáveis
  changes.md                       Propostas de robustez avaliadas e não aplicadas
relatorio/
  relatorio.tex                    Relatório final (IEEEtran, PT-BR) — compilar no Overleaf
  imagem/                          Figuras usadas no relatório
final_project/
  limo_bebop.m                     Cópia da versão submetida (repositório próprio no GitHub)
external/
  formacion_limo_bebop_final.m     Script de referência de outro grupo, usado para comparação
```

## Controle em hardware: MATLAB, ROS e OptiTrack

### Pré-requisitos de laboratório

1. Inicie o ROS master em `192.168.0.100`.
2. Inicie a ponte OptiTrack (`natnet_ros`).
3. No Motive, configure os corpos rígidos `L1` e `B1`.
4. Inicie o LIMO no modo diferencial, no namespace `L1`.
5. Inicie o driver do Bebop no namespace `B1`.
6. Conecte o joystick antes de executar o script.

| Recurso | Tópico |
| --- | --- |
| Pose LIMO | `/natnet_ros/L1/pose` |
| Comando LIMO | `/L1/cmd_vel` |
| Pose Bebop | `/natnet_ros/B1/pose` |
| Comando Bebop | `/B1/cmd_vel` |
| Decolagem / pouso | `/B1/takeoff`, `/B1/land` |

### Executando por seções

`limo_bebop_final.m` é dividido em seções do MATLAB (`%%`, executáveis isoladamente com *Run
Section*): parâmetros, inicialização do ROS/joystick, variáveis de estado, decolagem manual,
loop de controle, pouso manual e resultados. Isso evita reconectar ao ROS e reler as poses a cada
ajuste de ganho durante os testes de bancada.

### Joystick

- **Botão A**: decola o Bebop e inicia a formação.
- **Botão B**: para tudo e pousa (emergência), a qualquer momento.
- **Analógico**: sobrescreve o comando do Bebop assim que sai de zero, sem precisar de botão —
  o controle automático retoma sozinho quando o analógico volta ao centro.
- Na rotina de pouso (automática ou manual), há uma janela de alguns segundos em que o piloto
  pode afastar o drone do LIMO pelo joystick antes do comando de pouso ser enviado — necessário
  porque a posição de equilíbrio da formação fica próxima da vertical do LIMO.

### Sequência segura recomendada

1. Rode `teste_limo_comando_fixo.m` / `teste_limo_lemniscata.m` para validar o LIMO sozinho.
2. Rode `teste_convergencia_bebop.m` com o Bebop pousado/segurado, sem enviar nenhum comando, só
   para conferir se o alvo e o erro calculados fazem sentido.
3. Rode `limo_bebop_final.m` com o Bebop desligado (mas tracked pelo OptiTrack) para conferir os
   comandos calculados sem risco.
4. Primeiro voo apenas em área livre, com o Botão B (parada de emergência) sempre acessível.

## Auditoria e visualização

Com `cfg.audit_enabled = true`, o script grava um arquivo `results/limo_bebop_final/audit_*.txt`
por execução, com alvo, erro, comandos e sinais intermediários por robô. Para gerar gráficos e
uma animação a partir de um audit:

```bash
uv run python sim/visualizar_auditoria_formacao.py \
  results/limo_bebop_final/audit_YYYYMMDD_HHMMSS.txt --gif
```

## Segurança implementada

- botão de decolagem e botão de parada de emergência no joystick;
- bloco `try/catch` que pousa o drone em caso de erro em tempo de execução;
- parede virtual configurável por direção (`±x`, `±y`, `+z`);
- *watchdog* de perda de corpo rígido no OptiTrack (`> 0,5 s`), baseado no *timestamp* da pose;
- sobrescrita manual do comando pelo analógico do joystick, a qualquer momento;
- janela de afastamento manual do drone antes do pouso automático.

## Relatório

O relatório final está em [`relatorio/relatorio.tex`](relatorio/relatorio.tex) (classe
`IEEEtran`, compilar no Overleaf com a pasta `relatorio/imagem/`). Ele documenta os parâmetros
efetivamente usados no ensaio de voo, as métricas de erro de formação e de rastreio, e as
decisões de implementação (ajuste de `β_f`, estrutura em seções, sobrescrita manual).

## Limitações conhecidas

- A referência de altitude do Bebop é somada ao `z` medido do LIMO em vez de assumida como zero;
  isso reduziu a folga vertical contra a parede virtual no último ensaio (`~6 cm`) e deve ser
  revisado antes de aumentar `β_f`.
- O laço interno do LIMO usa um estado de velocidade integrado internamente, não a velocidade
  medida via diferença finita da pose — decisão para evitar ruído do OptiTrack amplificado pela
  diferenciação a 30 Hz, mas que reduz a robustez teórica do compensador a divergências entre o
  modelo identificado e o robô real.
