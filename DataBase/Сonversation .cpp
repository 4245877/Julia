#include <iostream>
#include <libpq-fe.h> // �������� ������������ ���� PostgreSQL

void checkConn(PGconn* conn) {
    if (PQstatus(conn) != CONNECTION_OK) {
        std::cerr << "������ �����������: " << PQerrorMessage(conn) << std::endl;
        PQfinish(conn);
        exit(1);
    }
}

