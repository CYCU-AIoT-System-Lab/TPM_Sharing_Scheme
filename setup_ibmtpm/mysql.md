# Usage of MySQL

## Note

1. SQL commands can be both lower and upper case.
2. SQL commands needs to be ends with semicolon ```;```.

## Usage

1. Administrate MySQL DB: ```sudo mysql -u root```
    1. Check available DBs: ```show databases;```
    2. Create DB: ```create database <DBname>;```
    3. Create DB if not exist: ```create database if not exists <DBname>;```
    4. Enter/Use DB: ```use <DBname>;```
        1. Show tables in DB: ```show tables;```
        2. Create table in DB: ```create table <TABLEname> (<col1_name> <col1_type>, <col2_name> <col2_type>)```
            1. Column type Integer -> ```int```
            2. Column type String -> ```varchar(<length>)```
            3. Example: ```create table users (id int, name varchar(20), email varchar(20));```
        3. Add data to table in DB: ```insert into <TABLEname> (<col1_name>, <col2_name>) values (<col1_value>, <col2_value>);```
            1. Integer value -> ```123```
            2. String value -> ```"abc"```
            3. Not all column in table is required.
            4. Example: ```insert into users (id, name) values (1, "belongtothenight");```
        4. Display data in table in DB: ```select * from <TABLEname>```
            1. Example: ```select * from users```
        5. No need to exit a DB, just use/enter different DB with command.

## Link for further details

1. POPSQL Learn-SQL of PostgreSQL, Redshift, MySQL, SQL Server, BigQuery, Snowflake operations. <https://popsql.com/learn-sql/mysql/how-to-insert-in-mysql>
2. W3 school MySQL Tutorial: <https://www.w3schools.com/mysql/default.asp>
3. MySQL official documentation: <https://dev.mysql.com/doc/refman/8.0/en/preface.html>
