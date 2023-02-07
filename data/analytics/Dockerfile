FROM domsoos/ridedb

ENV MYSQL_DATABASE=ridedb \
    MYSQL_ROOT_PASSWORD=411Team0r4ng3 \
    MYSQL_ROOT_HOST=%

ADD schema.sql /docker-entrypoint-initdb.d

EXPOSE 3306
