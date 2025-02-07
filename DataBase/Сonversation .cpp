#include <iostream>
#include <libpq-fe.h> // Основной заголовочный файл PostgreSQL

void checkConn(PGconn* conn) {
    if (PQstatus(conn) != CONNECTION_OK) {
        std::cerr << "Ошибка подключения: " << PQerrorMessage(conn) << std::endl;
        PQfinish(conn);
        exit(1);
    }
}

