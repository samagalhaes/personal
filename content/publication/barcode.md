+++
title = "Leitura de código de barras"
date = "2016-12-15"

# Authors. Comma separated list, e.g. `["Bob Smith", "David Jones"]`.
authors = ["Sandro Magalhães", "Tiago Mendonça"]

# Publication type.
# Legend:
# 0 = Uncategorized
# 1 = Conference proceedings
# 2 = Journal
# 3 = Work in progress
# 4 = Technical report
# 5 = Book
# 6 = Book chapter
publication_types = ["2"]

# Publication name and optional abbreviated version.
publication = "In *Sistemas Baseados em Visão*, FEUP."
publication_short = "In *SBVI - FEUP*"

# Abstract and optional shortened version.
abstract = "Desenvolveu-se um *script* que seja capaz de desencriptar códigos de barras, recorrendo ao *software* MatLab, nomeadamente, à *toolbox* de processamento de imagem."
abstract_short = "Descodificação de códigos de barras 2D utilizando MatLab"

# Featured image thumbnail (optional)
image_preview = ""

# Is this a selected publication? (true/false)
selected = false

# Projects (optional).
#   Associate this publication with one or more of your projects.
#   Simply enter the filename (excluding '.md') of your project file in `content/project/`.
#projects = ["deep-learning"]

# Links (optional).
url_pdf = "files/pt/feup/sbvi/report.pdf"
#url_preprint = ""
url_code = "https://github.com/samagalhaes/Barcode-Scanner"
#url_dataset = ""
#url_project = ""
url_slides = "files/pt/feup/sbvi/slides.pdf"
#url_video = ""
#url_poster = ""
#url_source = ""

# Custom links (optional).
#   Uncomment line below to enable. For multiple links, use the form `[{...}, {...}, {...}]`.
url_custom = [{name = "Sigarra FEUP", url = "https://sigarra.up.pt/feup/pt/ucurr_geral.ficha_uc_view?pv_ocorrencia_id=385632"}]

# Does the content use math formatting?
math = true

# Does the content use source code highlighting?
highlight = true

# Featured image
# Place your image in the `static/img/` folder and reference its filename below, e.g. `image = "example.jpg"`.
[header]
image = ""
caption = ""

+++

O objetivo do trabalho prendia-se com a leitura de um código de barras com recurso aos fundamentos teóricos subjacentes às técnicas de processamento de imagem e fazendo uso das ferramentas disponibilizadas pela Image Processing Toolbox do MatLab.

O processamento da imagem contemplou várias fases. Realizaram-se operações de pré-processamento no sentido de a suavizar, procedeu-se à sua reorientação, fez-se uso de técnicas de segmentação, isolando as regiões mais relevantes, e, por fim, descodificaram-se as barras extraídas.

Finalmente, os resultados obtidos permitiram validar o algoritmo implementado, constatando-se que a descodificação era total para grande parte das imagens utilizadas com ângulo de inclinação situado entre −90 e +90.