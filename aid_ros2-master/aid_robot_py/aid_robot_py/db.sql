PRAGMA foreign_keys= OFF;
BEGIN TRANSACTION;
CREATE TABLE map
(
    id               integer primary key autoincrement,
    name             text,
    file_path        text,
    create_timestamp int,
    edit_timestamp   int,
    creator_id       int
);
CREATE TABLE waypoint
(
    id               integer primary key autoincrement,
    map_id           integer,
    point_list       text,
    frame_id         text,
    create_timestamp int,
    edit_timestamp   int,
    creator_id       int
);
CREATE TABLE forbidden
(
    id               integer primary key autoincrement,
    map_id           integer,
    point_list       text,
    frame_id         text,
    create_timestamp int,
    edit_timestamp   int,
    creator_id       int
);
CREATE TABLE current_map
(
    id        integer,
    file_path text,
    name      text
);
DELETE
FROM sqlite_sequence;
COMMIT;
