# Saved model for weight agent to select between weight modes.
package(default_visibility = [
    "//visibility:public",  # For anyone to use.
])

filegroup(
    name = "saved_model",
    data = [
        "keras_metadata.pb",
        "saved_model.pb",
    ] + glob([
        "variables/*",
        "assets.extra/*",
    ]),
)
