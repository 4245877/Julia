#include <iostream>
#include <libpq-fe.h> // �������� ������������ ���� PostgreSQL

void checkConn(PGconn* conn) {
    if (PQstatus(conn) != CONNECTION_OK) {
        std::cerr << "������ �����������: " << PQerrorMessage(conn) << std::endl;
        PQfinish(conn);
        exit(1);
    }
}

int main() {
    // ������ ��� �����������
    const char* conninfo = "dbname=testdb user=postgres password=yourpassword hostaddr=127.0.0.1 port=5432";
    
    // ������������ � ��
    PGconn* conn = PQconnectdb(conninfo);
    checkConn(conn);
    std::cout << "����������� �������!" << std::endl;

    // ��������� SQL-������
    PGresult* res = PQexec(conn, "SELECT id, name FROM users");
    if (PQresultStatus(res) != PGRES_TUPLES_OK) {
        std::cerr << "������ ���������� �������: " << PQerrorMessage(conn) << std::endl;
        PQclear(res);
        PQfinish(conn);
        return 1;
    }

    // ������� ���������
    int rows = PQntuples(res);
    for (int i = 0; i < rows; i++) {
        std::cout << "ID: " << PQgetvalue(res, i, 0) << ", Name: " << PQgetvalue(res, i, 1) << std::endl;
    }

    // ������� ��������
    PQclear(res);
    PQfinish(conn);

    return 0;
}
