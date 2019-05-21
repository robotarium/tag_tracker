# https://docs.bazel.build/versions/master/be/c-cpp.html#cc_binary
cc_binary(
    name = "tag_tracker",
    srcs = ["new.cc"],
    copts = [
        "-Iexternal/gtest/include"
    ],
    deps = [
        "@vizier//vizier/vizier_node:vizier_node",
    ],
)