# A-Star

Implementação em C++ do algoritmo A* para busca de caminhos em um grid (console).

## Sobre o projeto

Este repositório contém uma implementação didática e autônoma do algoritmo A* em C++.
O programa gera um mapa em grade (grid) com obstáculos e custos de terreno, e calcula
um caminho entre um ponto inicial (S) e um objetivo (G) exibindo o resultado em ASCII
no terminal. O código é voltado para fins educacionais e experimentação com heurísticas
e custo de movimento.

## Tecnologias utilizadas

- Linguagem: C++ (uso de recursos da biblioteca padrão: <vector>, <random>, <queue>, etc.)
- Compilador: `g++` (comandos de compilação fornecidos em `comandos.txt`)

Nenhuma dependência externa foi encontrada no repositório.

## Arquitetura / Estrutura do projeto

Estrutura principal do repositório:

```
A-Star/
├─ comandos.txt         # comandos de build/execução fornecidos
├─ astar.exe            # (opcional) executável pré-compilado (se presente)
├─ README.md
└─ src/
   ├─ astar.h
   ├─ astar.cpp
   └─ main.cpp
```

- `src/astar.h` — declarações das estruturas (`Position`, `PathResult`) e da classe `astar::Solver`.
- `src/astar.cpp` — implementação do algoritmo A* (fila de prioridade, cálculo de custos, reconstrução do caminho).
- `src/main.cpp` — aplicação console que prepara mapas (determinístico e aleatório), executa o solver e imprime o resultado em formato ASCII.
- `comandos.txt` — exemplo simples de comandos para compilar e executar o projeto.

## Funcionalidades implementadas

- Implementação funcional do algoritmo A* para grades 2D.
- Geração de dois tipos de mapa:
  - Mapa determinístico (fixo) com obstáculos e variação de custo de terreno.
  - Mapa aleatório com probabilidade de obstáculos e custos randômicos.
- Cálculo de custo total do caminho e reconstrução do trajeto.
- Impressão do mapa e do caminho no terminal usando caracteres ASCII:
  - `S` = start, `G` = goal, `P` = caminho, `#` = obstáculo, `g` = grama (custo 2), `M` = montanha (custo 5).

As funcionalidades acima são implementadas pelo código fonte; não há servidor, API, banco de dados ou testes automatizados no repositório.

## Pré-requisitos

- Um compilador C++ compatível com C++11 ou superior (ex.: `g++`).
- Ferramenta de linha de comando (Terminal / CMD / PowerShell).

Observação: o código utiliza `<random>` e construções da STL modernas, portanto C++11 é suficiente.

## Instalação

1. Clone o repositório:

```bash
git clone <repositório>
cd A-Star
```

2. Compile usando `g++` (comando provido em `comandos.txt`):

```bash
g++ src/main.cpp src/astar.cpp -o astar
```

No Windows com MinGW/DevKit/WSL, substitua `g++` pela cadeia de ferramentas apropriada se necessário.

## Configuração

Não existem arquivos de configuração, variáveis de ambiente ou credenciais no repositório. O programa é executado como aplicativo de console e não requer configuração adicional.

## Execução

Após compilar, execute:

```bash
# Linux / macOS
./astar

# Windows (cmd / PowerShell)
.\astar.exe
```

O programa perguntará qual mapa usar (digite `1` para mapa determinístico ou `2` para mapa randômico). Em seguida exibirá:

- mensagem de sucesso/erro (caminho encontrado ou não);
- custo total do caminho;
- número de passos;
- mapa ASCII com a rota destacada.

Você também pode executar o executável pré-compilado `astar.exe` caso ele esteja presente na raiz do repositório.

## Desenvolvimento

- Para modificar o algoritmo, edite `src/astar.cpp` e `src/astar.h`.
- Para alterar o cenário/mapa, edite as funções `createGrid()` e `createRandomGrid()` em `src/main.cpp`.
- Recomenda-se compilar com otimizações e habilitar warnings para desenvolvimento:

```bash
g++ -std=c++11 -Wall -Wextra -O2 src/main.cpp src/astar.cpp -o astar
```

## Status do projeto

- Estado atual: funcional e focado em demonstração/educação. O repositório contém implementação completa do A* e uma pequena interface de console.
- Testes automatizados: ausentes.
- Integrações: ausentes.


---

### Observações finais

- O arquivo `comandos.txt` contém um comando direto para compilar e executar o projeto; ele pode ser utilizado como referência rápida.
- Nenhuma informação sensível (chaves, tokens, credenciais) foi encontrada.
- Se desejar, posso:
  - adicionar instruções para executar com diferentes compiladores/IDE;
  - incluir um arquivo `Makefile` ou `CMakeLists.txt` para facilitar builds;
  - adicionar testes unitários simples para `astar::Solver`.
