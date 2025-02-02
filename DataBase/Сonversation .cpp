#include <iostream>
#include <libpq-fe.h> // Основной заголовочный файл PostgreSQL

void checkConn(PGconn* conn) {
    if (PQstatus(conn) != CONNECTION_OK) {
        std::cerr << "Ошибка подключения: " << PQerrorMessage(conn) << std::endl;
        PQfinish(conn);
        exit(1);
    }
}

int main() {
    // Данные для подключения
    const char* conninfo = "dbname=testdb user=postgres password=yourpassword hostaddr=127.0.0.1 port=5432";
    
    // Подключаемся к БД
    PGconn* conn = PQconnectdb(conninfo);
    checkConn(conn);
    std::cout << "Подключение успешно!" << std::endl;

    // Выполняем SQL-запрос
    PGresult* res = PQexec(conn, "SELECT id, name FROM users");
    if (PQresultStatus(res) != PGRES_TUPLES_OK) {
        std::cerr << "Ошибка выполнения запроса: " << PQerrorMessage(conn) << std::endl;
        PQclear(res);
        PQfinish(conn);
        return 1;
    }

    // Выводим результат
    int rows = PQntuples(res);
    for (int i = 0; i < rows; i++) {
        std::cout << "ID: " << PQgetvalue(res, i, 0) << ", Name: " << PQgetvalue(res, i, 1) << std::endl;
    }

    // Очистка ресурсов
    PQclear(res);
    PQfinish(conn);

    return 0;
}
