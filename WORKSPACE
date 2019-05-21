load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "vizier",
    urls = ["https://github.com/robotarium/vizier_cpp/archive/master.zip"],
    sha256 = "5b8e0650cbe78b22897f118f5aa336084252e93cf2904043e5c37a3228264919",
    strip_prefix = "vizier_cpp-master",
)
### VIZIER DEPS 

http_archive(
    name = "spdlog",
    urls = ["https://github.com/gabime/spdlog/archive/v1.x.zip"],
    sha256 = "7d4fad22a47952266c8ae107b1430272f79607a7308ea42f43eea7e977386bdd",
    strip_prefix = "spdlog-1.x",
    build_file = "@vizier//:spdlog.BUILD",
)

http_archive(
    name = "json",
    urls = ["https://github.com/nlohmann/json/archive/develop.zip"],
    sha256 = "562ae414ed555c1540af322076a4a9eccd68e0133c74ba8a6f84e70d494dd3ed",
    strip_prefix = "json-develop",
    build_file = "@vizier//:njson.BUILD",
)

http_archive(
    name = "gtest",
    url = "https://github.com/google/googletest/archive/release-1.7.0.zip",
    sha256 = "b58cb7547a28b2c718d1e38aee18a3659c9e3ff52440297e965f5edffe34b6d0",
    build_file = "@vizier//:gtest.BUILD",
    strip_prefix = "googletest-release-1.7.0",
)

### END VIZIER DEPS