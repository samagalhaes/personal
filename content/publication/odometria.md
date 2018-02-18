+++
title = "Odometria e equações de movimento de um robô"
date = "2017-09-28"

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
abstract = "Este relatório surge no âmbito do trabalho prático para a unidade curricular de Sistemas Robóticos Autónomos, realizado em sala de aula no dia 27 de setembro de 2017, que teve como principal intuito incentivar os estudantes a estudar o comportamento de uma robot de tração diferencial com duas rodas e calcular alguns parâmetros das características inerentes."
abstract_short = "Calculo dos parâmetros da odometria para um robô móvel de tração diferencial, recorrendo ao *software* de simulação SimTwo"

# Featured image thumbnail (optional)
image_preview = ""

# Is this a selected publication? (true/false)
selected = false

# Projects (optional).
#   Associate this publication with one or more of your projects.
#   Simply enter the filename (excluding '.md') of your project file in `content/project/`.
#projects = ["deep-learning"]

# Links (optional).
url_pdf = "files/pt/feup/srau/TP1/relat_tp1_SandroTiago.pdf"
#url_preprint = ""
url_code = "files/pt/feup/srau/TP1/SRAU_SandroTiago.rar"
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
Este relatório surge no âmbito do trabalho prático para a unidade curricular de Sistemas
Robóticos Autónomos, realizado em sala de aula no dia 27 de setembro de 2017, que teve como
principal intuito incentivar os estudantes a estudar o comportamento de uma robot de tração
diferencial com duas rodas e calcular alguns parâmetros das características inerentes.

Como ambiente de trabalho para a realização do exercício, recorreu-se ao simulador
SimTwo que disponibiliza um modelo do robot supra indicado. Este desloca-se num plano
horizontal intermédio de duas rodas motrizes diferenciais e com uma terceira roda livre de
apoio.

Como ponto de partida, teve-se em consideração as equações base que permitem o
controlo da velocidade linear ($v$) e velocidade angular ($\omega$), por controlo da velocidade dos
motores de cada roda ($v_1$ e $v_2$ ).

$$
    \begin{equation}
        v &= \dfrac{v_1 + v_2}{2} \\
    \end{equation}
$$

$$
    \begin{equation}
        \omega &= \dfrac{v_1 - v_2}{2 \cdot b}\\
    \end{equation}
$$