echo n=%1
set /A port1=31001 + %1
set /A port2=31002 + %1

codeball2018.exe --p1 tcp-%port1% --p2 tcp-%port2% --noshow --results-file result-%1.txt --nitro true --seed %1 -new_console
ping 192.168.2.1 -w 4000 -n 1
CodeBall.exe 127.0.0.1 %port1% 0 -new_console
CodeBallV6.exe 127.0.0.1 %port2% 0 -new_console
