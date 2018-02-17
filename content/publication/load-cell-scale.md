+++
title = "Medição de Massa (Balança)"
date = "2015-09-01"

# Authors. Comma separated list, e.g. `["Bob Smith", "David Jones"]`.
authors = ["Sandro Magalhães", "Sérgio Fernandes"]

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
publication = "In *Medição Sensores e Instrumentação*, FEUP."
publication_short = "In *MSIN - FEUP*"

# Abstract and optional shortened version.
abstract = "Neste projecto implementou-se uma cadeia de medição – balança – cuja mensuranda é a massa. O sinal foi adquirido e condicionado no seu formato analógico e, posteriormente, convertido para digital onde, cujo valor DC, foi transformado, após um processo de calibração, num valor expresso em gramas."
abstract_short = "Medição de massa utilizando uma célula de carga. O sinal foi processado no software LabView."

# Featured image thumbnail (optional)
image_preview = ""

# Is this a selected publication? (true/false)
selected = false

# Projects (optional).
#   Associate this publication with one or more of your projects.
#   Simply enter the filename (excluding '.md') of your project file in `content/project/`.
#projects = ["deep-learning"]

# Links (optional).
url_pdf = "/files/pt/feup/msin/load-cell-scale.pdf"
#url_preprint = ""
#url_code = ""
url_dataset = "/files/pt/feup/msin/load-cell-scale.zip"
#url_project = ""
#url_slides = ""
#url_video = ""
#url_poster = ""
#url_source = ""

# Custom links (optional).
#   Uncomment line below to enable. For multiple links, use the form `[{...}, {...}, {...}]`.
url_custom = [{name = "Sigarra FEUP", url = "https://sigarra.up.pt/feup/pt/UCURR_GERAL.FICHA_UC_VIEW?pv_ocorrencia_id=365703"}]

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

No âmbito da unidade curricular de medição, sensores e instrumentação, proposemo-nos à elaboração de uma cadeia de medição para a construção de uma balança cuja mensuranda será a massa.

Para iniciar este projecto foi-nos disponibilizada a escolha entre uma régua para medir a massa ou uma célula de carga – tendo nós escolhido a segunda opção, que já estava devidamente montada com os quatro extensómetros e respectivas conexões.

Além do circuito de aquisição e condicionamento do sinal que deverá ser feito de forma analógica, todo o resto do projecto deverá ser implementado no software LabView que fornece todas as ferramentas de cálculo e funções para o posterior tratamento do sinal digital para o qual será convertido através de uma placa de aquisição montada ao computador das bancada do laboratório.
