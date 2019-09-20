# IB1
Instrumentação Biomédica 1
[Cronograma](https://docs.google.com/spreadsheets/d/1-90X5oCXaSawRg9z0GiOyt68x7-coC0qeFPKeV69EhE/edit#gid=0)

## Organização dos Arquivos
* **dacq_app**: Software de acquisição.

## Instalando
* Instale o [Python](https://www.python.org/) caso seu computador ja não tenha. `python --version`
* Instale o virtual env: `pip install virtualenv` [Documentação](https://virtualenv.pypa.io/en/latest/installation/)
* Crie um ambiente virtual: `virtualenv env`
* Ative o ambiente virual: `source env/bin/activate`
* Instale as bibliotecas requeridas: `pip install -r requirements.txt`
* Caso altere a lista de bibliotecas utilize `c` para atualizar o arquivo.

## Rodando
* Abra um terminal na pasta *dacq_app*.
* Execute o arquivo **app.py**: `python app.py`
