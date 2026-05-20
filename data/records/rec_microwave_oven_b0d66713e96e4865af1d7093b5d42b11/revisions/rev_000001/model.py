from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_microwave")

    stainless = model.material("stainless", color=(0.73, 0.74, 0.76, 1.0))
    dark_glass = model.material("dark_glass", color=(0.10, 0.12, 0.14, 0.92))
    charcoal = model.material("charcoal", color=(0.18, 0.19, 0.21, 1.0))
    black_plastic = model.material("black_plastic", color=(0.08, 0.08, 0.09, 1.0))
    cavity_gray = model.material("cavity_gray", color=(0.82, 0.83, 0.85, 1.0))
    screen_green = model.material("screen_green", color=(0.30, 0.55, 0.34, 1.0))

    body_w = 0.51
    body_d = 0.34
    body_h = 0.31
    shell_t = 0.012
    front_t = 0.014

    opening_left = -0.235
    opening_right = 0.117
    opening_bottom = 0.048
    opening_top = 0.266

    door_w = 0.350
    door_h = 0.216
    door_t = 0.018

    separator_w = 0.018
    left_bezel_w = 0.020

    vent_x0 = opening_right + separator_w + 0.002
    vent_w = body_w / 2 - shell_t - vent_x0 - 0.002
    vent_top = 0.122
    vent_h = 0.066
    vent_bottom = vent_top - vent_h
    flap_t = 0.011

    body = model.part("body")
    body.visual(
        Box((body_w, body_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, shell_t / 2)),
        material=stainless,
        name="shell_bottom",
    )
    body.visual(
        Box((body_w, body_d, shell_t)),
        origin=Origin(xyz=(0.0, 0.0, body_h - shell_t / 2)),
        material=stainless,
        name="shell_top",
    )
    body.visual(
        Box((shell_t, body_d, body_h - 2 * shell_t)),
        origin=Origin(xyz=(-body_w / 2 + shell_t / 2, 0.0, body_h / 2)),
        material=stainless,
        name="shell_left",
    )
    body.visual(
        Box((shell_t, body_d, body_h - 2 * shell_t)),
        origin=Origin(xyz=(body_w / 2 - shell_t / 2, 0.0, body_h / 2)),
        material=stainless,
        name="shell_right",
    )
    body.visual(
        Box((body_w - 2 * shell_t, shell_t, body_h - 2 * shell_t)),
        origin=Origin(xyz=(0.0, -body_d / 2 + shell_t / 2, body_h / 2)),
        material=stainless,
        name="shell_back",
    )

    door_frame_xmin = opening_left - left_bezel_w
    door_frame_xmax = opening_right + separator_w
    door_frame_span = door_frame_xmax - door_frame_xmin

    body.visual(
        Box((left_bezel_w, front_t, body_h)),
        origin=Origin(
            xyz=(opening_left - left_bezel_w / 2, body_d / 2 - front_t / 2, body_h / 2)
        ),
        material=charcoal,
        name="left_bezel",
    )
    body.visual(
        Box((door_frame_span, front_t, opening_bottom)),
        origin=Origin(
            xyz=((door_frame_xmin + door_frame_xmax) / 2, body_d / 2 - front_t / 2, opening_bottom / 2)
        ),
        material=charcoal,
        name="bottom_bezel",
    )
    body.visual(
        Box((door_frame_span, front_t, body_h - opening_top)),
        origin=Origin(
            xyz=(
                (door_frame_xmin + door_frame_xmax) / 2,
                body_d / 2 - front_t / 2,
                opening_top + (body_h - opening_top) / 2,
            )
        ),
        material=charcoal,
        name="top_bezel",
    )
    body.visual(
        Box((separator_w, front_t, body_h)),
        origin=Origin(
            xyz=(opening_right + separator_w / 2, body_d / 2 - front_t / 2, body_h / 2)
        ),
        material=charcoal,
        name="door_separator",
    )

    control_face_w = body_w / 2 - shell_t - vent_x0
    control_face_x = vent_x0 + control_face_w / 2
    body.visual(
        Box((control_face_w, front_t, body_h - vent_top)),
        origin=Origin(
            xyz=(control_face_x, body_d / 2 - front_t / 2, vent_top + (body_h - vent_top) / 2)
        ),
        material=charcoal,
        name="control_face_upper",
    )
    body.visual(
        Box((control_face_w, front_t, vent_bottom)),
        origin=Origin(xyz=(control_face_x, body_d / 2 - front_t / 2, vent_bottom / 2)),
        material=charcoal,
        name="control_face_lower",
    )
    body.visual(
        Box((control_face_w * 0.70, 0.004, 0.034)),
        origin=Origin(xyz=(control_face_x, body_d / 2 - 0.002, 0.246)),
        material=screen_green,
        name="display_window",
    )
    body.visual(
        Box((control_face_w * 0.78, 0.004, 0.094)),
        origin=Origin(xyz=(control_face_x, body_d / 2 - 0.002, 0.174)),
        material=black_plastic,
        name="button_bank",
    )
    body.visual(
        Box((door_w - 0.050, 0.010, door_h - 0.042)),
        origin=Origin(
            xyz=(
                (opening_left + opening_right) / 2,
                -body_d / 2 + shell_t + 0.005,
                opening_bottom + (door_h - 0.042) / 2,
            )
        ),
        material=cavity_gray,
        name="cavity_backdrop",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_w, body_d, body_h)),
        mass=11.5,
        origin=Origin(xyz=(0.0, 0.0, body_h / 2)),
    )

    door = model.part("door")
    door.visual(
        Box((door_w, door_t, door_h)),
        origin=Origin(xyz=(door_w / 2, door_t / 2, door_h / 2)),
        material=charcoal,
        name="door_panel",
    )
    door.visual(
        Box((door_w - 0.072, door_t * 0.56, door_h - 0.084)),
        origin=Origin(xyz=(door_w / 2 - 0.004, door_t * 0.70, door_h / 2 + 0.003)),
        material=dark_glass,
        name="door_window",
    )
    door.visual(
        Box((door_w - 0.050, door_t * 0.52, door_h - 0.056)),
        origin=Origin(xyz=(door_w / 2, door_t * 0.26, door_h / 2)),
        material=cavity_gray,
        name="door_inner_liner",
    )
    door.visual(
        Box((0.014, 0.028, 0.016)),
        origin=Origin(xyz=(door_w - 0.012, 0.024, door_h * 0.72)),
        material=black_plastic,
        name="door_handle_upper_mount",
    )
    door.visual(
        Box((0.014, 0.028, 0.016)),
        origin=Origin(xyz=(door_w - 0.012, 0.024, door_h * 0.28)),
        material=black_plastic,
        name="door_handle_lower_mount",
    )
    door.visual(
        Cylinder(radius=0.008, length=door_h * 0.42),
        origin=Origin(xyz=(door_w - 0.012, 0.038, door_h / 2)),
        material=black_plastic,
        name="door_handle_grip",
    )
    door.visual(
        Cylinder(radius=0.006, length=door_h * 0.94),
        origin=Origin(xyz=(0.0, door_t / 2, door_h / 2)),
        material=black_plastic,
        name="door_hinge_barrel",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_w, door_t, door_h)),
        mass=2.1,
        origin=Origin(xyz=(door_w / 2, door_t / 2, door_h / 2)),
    )

    vent_flap = model.part("vent_flap")
    vent_flap.visual(
        Box((vent_w, flap_t, vent_h)),
        origin=Origin(xyz=(vent_w / 2, flap_t / 2, -vent_h / 2)),
        material=black_plastic,
        name="vent_panel",
    )
    vent_flap.visual(
        Cylinder(radius=0.004, length=vent_w),
        origin=Origin(xyz=(vent_w / 2, flap_t / 2, 0.0), rpy=(0.0, pi / 2, 0.0)),
        material=black_plastic,
        name="vent_hinge_barrel",
    )
    vent_flap.visual(
        Box((vent_w * 0.48, flap_t * 1.5, 0.012)),
        origin=Origin(xyz=(vent_w / 2, flap_t * 0.85, -vent_h + 0.006)),
        material=charcoal,
        name="vent_pull_lip",
    )
    vent_flap.inertial = Inertial.from_geometry(
        Box((vent_w, flap_t, vent_h)),
        mass=0.15,
        origin=Origin(xyz=(vent_w / 2, flap_t / 2, -vent_h / 2)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(opening_left, body_d / 2, opening_bottom)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=2.0,
            lower=0.0,
            upper=1.95,
        ),
    )
    model.articulation(
        "body_to_vent_flap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=vent_flap,
        origin=Origin(xyz=(vent_x0, body_d / 2, vent_top)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    vent_flap = object_model.get_part("vent_flap")
    door_hinge = object_model.get_articulation("body_to_door")
    flap_hinge = object_model.get_articulation("body_to_vent_flap")

    ctx.check("body part exists", body is not None)
    ctx.check("door part exists", door is not None)
    ctx.check("vent flap part exists", vent_flap is not None)
    ctx.check(
        "door hinge uses a vertical axis",
        tuple(door_hinge.axis) == (0.0, 0.0, 1.0),
        details=f"axis={door_hinge.axis}",
    )
    ctx.check(
        "vent flap hinge uses a horizontal axis",
        tuple(flap_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={flap_hinge.axis}",
    )

    ctx.expect_gap(
        door,
        body,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="door_panel",
        name="door sits flush against the front opening",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="xz",
        min_overlap=0.18,
        elem_a="door_panel",
        name="door covers the cooking cavity opening",
    )
    ctx.expect_gap(
        vent_flap,
        body,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="vent_panel",
        name="vent flap closes flush with the front face",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_hinge: 1.45}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        ctx.expect_gap(
            door,
            body,
            axis="y",
            min_gap=-1e-6,
            positive_elem="door_panel",
            name="open door stays outside the microwave body",
        )
    ctx.check(
        "door swings outward from the side hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][1] > closed_door_aabb[1][1] + 0.10,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_flap_aabb = ctx.part_element_world_aabb(vent_flap, elem="vent_panel")
    with ctx.pose({flap_hinge: 1.05}):
        open_flap_aabb = ctx.part_element_world_aabb(vent_flap, elem="vent_panel")
    ctx.check(
        "vent flap folds down and outward from its top hinge",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[1][1] > closed_flap_aabb[1][1] + 0.030
        and (open_flap_aabb[1][2] - open_flap_aabb[0][2])
        < (closed_flap_aabb[1][2] - closed_flap_aabb[0][2]) - 0.015
        and open_flap_aabb[0][2] > closed_flap_aabb[0][2] + 0.020,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
