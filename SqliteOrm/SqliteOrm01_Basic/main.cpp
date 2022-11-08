#include <fmt/format.h>
#include <sqlite_orm/sqlite_orm.h>
#include <common/common.hpp>
#include <iostream>

using namespace std;
using namespace fmt;
using namespace common;
using namespace sqlite_orm;

struct User {
    int id;
    string firstName;
    string lastName;
    int birthDate;
    std::unique_ptr<string> imageUrl;
    int typeId;
};

struct UserType {
    int id;
    string name;
};

void basic() {
    LOG(INFO) << Section("Basic", false);

    // create db
    auto storage =
        make_storage("./data/sqlorm.db",
                     make_table("Users", make_column("ID", &User::id, autoincrement(), primary_key()),
                                make_column("FirstName", &User::firstName), make_column("LastName", &User::lastName),
                                make_column("BirthDate", &User::birthDate), make_column("ImageUrl", &User::imageUrl),
                                make_column("TypeId", &User::typeId)),
                     make_table("UserTypes", make_column("ID", &UserType::id, autoincrement(), primary_key()),
                                make_column("Name", &UserType::name, default_value("None"))));
    storage.sync_schema();

    // insert
    User user{-1, "John", "Doe", 664416000, std::make_unique<string>("url to heaven"), 3};
    auto insertedId = storage.insert(user);
    LOG(INFO) << format("inserted ID for first user = {}", insertedId);
    user.id = insertedId;
    User secondUser{-1, "Alice", "Inwonder", 831168000, {}, 2};
    insertedId = storage.insert(secondUser);
    secondUser.id = insertedId;
    LOG(INFO) << format("inserted ID for second user = {}", insertedId);

    // update
    if (auto user = storage.get_pointer<User>(insertedId)) {
        LOG(INFO) << format("find user: ID = {}, {}, {}", insertedId, user->firstName, user->lastName);
        user->imageUrl =
            make_unique<string>("https://cdn1.iconfinder.com/data/icons/man-icon-set/100/man_icon-21-512.png");
        storage.update(*user);
    } else {
        LOG(WARNING) << format("not user with ID = {}", insertedId);
    }
}

int main(int argc, const char* argv[]) {
    initLog(argc, argv);

    basic();

    closeLog();
    return 0;
}