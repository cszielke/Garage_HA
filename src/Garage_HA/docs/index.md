# IBC Regenwasser Container Steuerung und Füllstandsüberwachung

Diese Dokumentation wurde mit MkDocs erstellt. Bitte bei Änderungen nicht die Html-Seiten auf dem Webserver ändern, sondern die **"*.md"** Datei im Unterverzeichnis **"docs"** der Quelldateien ändern.

Die Quelldateien befinden sich im Verzeichnis **docs** des Repositorys
## Serve
Mit dem Kommando 
```sh
mkdocs serve
```
wird ein Webserver gestartet und die erzeugten Doc-Html Dateien können mit dem Webbrowser unter der Adresse [`http://localhost:8000/`](http://localhost:8000/) aufgerufen werden. Jede Änderung wird im Webbrowser direkt sichtbar. Mit `Ctrl-C` wird der Webserver beendet.

## Build
Mit dem Kommando 
```sh
mkdocs build
```
werden die Html-Dateien erzeugt und in das Webserver Verzeichnis `\\raspidbsrv\web\html\doku\NodeMCU` kopiert und kann dann im Webbrowser unter der Adresse [`https://192.168.15.107/doku/NodeMCU/`](https://192.168.15.107/doku/NodeMCU/) aufgerufen werden.
