# Publicação de TF do Frame Odom

Este pacote agora inclui a publicação da transformação TF entre o frame `odom` e o frame `base_footprint`, que é necessária para a navegação e localização do robô.

## O que foi adicionado

### 1. Nó de Publicação de TF (odom_tf_publisher_node.py)
- Nó customizado Python que publica a transformação `odom` -> `base_footprint`
- Configurável via parâmetros ROS2
- Taxa de publicação ajustável
- Posição e orientação inicial configuráveis

### 2. Configuração (config/odom_tf_config.yaml)
- Arquivo de configuração com parâmetros padrão
- Define posição inicial, orientação e taxa de publicação

### 3. Integração no Launch File
O arquivo `bringup_encoder.launch.py` foi atualizado com duas opções:

#### Opção 1: Static Transform Publisher (ATIVA por padrão)
```python
# Mais simples e eficiente para transformações estáticas
odom_tf_static = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
)
```

#### Opção 2: Nó Customizado (Disponível se necessário)
```python
# Para funcionalidades avançadas e configuração via parâmetros
odom_tf_publisher = Node(
    package='caramelo_bringup',
    executable='odom_tf_publisher_node',
    parameters=[PathJoinSubstitution([caramelo_bringup_path, "config", "odom_tf_config.yaml"])]
)
```

## Como usar

### Usando a configuração padrão (recomendado)
```bash
ros2 launch caramelo_bringup bringup_encoder.launch.py
```

### Para trocar para o nó customizado
1. Comente as linhas do `odom_tf_static` no arquivo de launch
2. Descomente as linhas do `odom_tf_publisher`
3. Mude a referência no LaunchDescription de `odom_tf_static` para `odom_tf_publisher`

### Para configurar parâmetros customizados
Edite o arquivo `config/odom_tf_config.yaml`:
```yaml
odom_tf_publisher:
  ros__parameters:
    initial_x: 1.0        # Posição X inicial (metros)
    initial_y: 2.0        # Posição Y inicial (metros)
    initial_z: 0.0        # Posição Z inicial (metros)
    publish_rate: 100.0   # Taxa de publicação (Hz)
```

## Verificação

Para verificar se a transformação está sendo publicada:
```bash
# Ver todas as transformações ativas
ros2 run tf2_tools view_frames

# Ver o gráfico da árvore TF
ros2 run rqt_tf_tree rqt_tf_tree

# Verificar transformação específica
ros2 run tf2_ros tf2_echo odom base_footprint
```

## Estrutura da Árvore TF

Após a implementação, a árvore TF deve ter a seguinte estrutura:
```
odom
└── base_footprint
    └── base_link
        └── ... (outros frames do robô)
```

O frame `odom` representa a referência global fixa, enquanto `base_footprint` representa a projeção do robô no chão.
