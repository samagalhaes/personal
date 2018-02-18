+++
title = "Implementação de rotinas básicas de controlo do movimento de um robô (FollowLine)"
date = "2017-11-07"

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
publication_types = ["4"]

# Publication name and optional abbreviated version.
publication = "In *Sistemas Robóticos Autónomos*, FEUP."
publication_short = "In *SRAU - FEUP*"

# Abstract and optional shortened version.
abstract = "No presente trabalho prático pretende-se que um robot móvel de tração diferencial seja capaz de seguir um segmento de reta, definido pelos seus pontos inicial e final, e que termine o seu movimento no ponto final estabelecido, com uma orientação equivalente à da reta. Para o efeito, em primeiro lugar, o objetivo consiste em aproximar o veículo ao ponto mais próximo da linha definida e, quando suficientemente próximo desta, iniciar o seu seguimento em sentido conveniente para atingir o ponto final. Posto isto, impõe-se a necessidade de implementação de dois controladores distintos: controlador para aproximação (Controlador A) e, posteriormente, comutação para controlador de seguimento (Controlador B). Estes controladores podem ser assumidos como dois estados distintos de uma máquina de estados, em que as transições são determinadas em função da distância ao ponto mais próximo da reta. A sua supervisão é conseguida com a implementação de um terceiro procedimento responsável pela comutação entre estados (FollowLine), estando, por isso, num nível hierarquicamente superior. Como premissa adicional do problema considera-se que o robot está sempre localizado entre os limites impostos pelos extremos do segmento."
abstract_short = "Implementação de uma estrutura de controlo de trajetórias para a um robô de tração diferencial, baseado no seguimento de uma linha recta."

# Featured image thumbnail (optional)
image_preview = ""

# Is this a selected publication? (true/false)
selected = false

# Projects (optional).
#   Associate this publication with one or more of your projects.
#   Simply enter the filename (excluding '.md') of your project file in `content/project/`.
#projects = ["deep-learning"]

# Links (optional).
url_pdf = "files/pt/feup/srau/TP2/TP2b/report_tp22_srau.pdf"
#url_preprint = ""
url_code = "files/pt/feup/srau/TP2/TP2b/FollowLine.spas.pas"
#url_dataset = ""
#url_project = ""
url_slides = ""
#url_video = ""
#url_poster = ""
#url_source = ""

# Custom links (optional).
#   Uncomment line below to enable. For multiple links, use the form `[{...}, {...}, {...}]`.
url_custom = [{name = "Sigarra FEUP", url = "https://sigarra.up.pt/feup/pt/ucurr_geral.ficha_uc_view?pv_ocorrencia_id=401695"},
              {name = "SimTwo", url = "https://paginas.fe.up.pt/~paco/wiki/index.php?n=Main.SimTwo"}]

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
