from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


BODY_WIDTH = 0.045
BODY_DEPTH = 0.018
BODY_HEIGHT = 0.040
BODY_OUTER_X = BODY_WIDTH * 0.5
BODY_FRONT_Y = -BODY_DEPTH * 0.5
BODY_TOP_Z = BODY_HEIGHT

LEFT_HOLE_X = -0.010
RIGHT_HOLE_X = 0.010
HOLE_Y = 0.0

BUTTON_OUTER_X = BODY_OUTER_X
BUTTON_Y = -0.001
BUTTON_Z = 0.017
BUTTON_TRAVEL = 0.0018


def _rounded_block_mesh(width: float, height: float, depth: float, radius: float, name: str):
    geom = ExtrudeGeometry(rounded_rect_profile(width, height, radius), depth)
    geom.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brass_padlock")

    brass = model.material("brass", rgba=(0.77, 0.61, 0.24, 1.0))
    brass_dark = model.material("brass_dark", rgba=(0.58, 0.45, 0.16, 1.0))
    shackle_steel = model.material("shackle_steel", rgba=(0.76, 0.78, 0.81, 1.0))
    keyway_dark = model.material("keyway_dark", rgba=(0.15, 0.16, 0.17, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_WIDTH, 0.019, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=brass_dark,
        name="base_lip",
    )
    body.visual(
        _rounded_block_mesh(0.036, 0.024, BODY_DEPTH, 0.0045, "padlock_body_core"),
        origin=Origin(xyz=(-0.0045, 0.0, 0.016)),
        material=brass,
        name="body_core",
    )

    # Right-side skin segments leave a real button pocket instead of embedding the button.
    body.visual(
        Box((0.0085, BODY_DEPTH, 0.0105)),
        origin=Origin(xyz=(0.01825, 0.0, 0.00875)),
        material=brass,
        name="button_pocket_lower_wall",
    )
    body.visual(
        Box((0.0085, BODY_DEPTH, 0.0115)),
        origin=Origin(xyz=(0.01825, 0.0, 0.02525)),
        material=brass,
        name="button_pocket_upper_wall",
    )
    body.visual(
        Box((0.0085, 0.0045, 0.0085)),
        origin=Origin(xyz=(0.01825, -0.00675, BUTTON_Z)),
        material=brass,
        name="button_pocket_front_wall",
    )
    body.visual(
        Box((0.0085, 0.0065, 0.0085)),
        origin=Origin(xyz=(0.01825, 0.00575, BUTTON_Z)),
        material=brass,
        name="button_pocket_rear_wall",
    )

    collar_z = 0.0335
    collar_h = 0.013
    side_wall_w = 0.003
    side_wall_d = 0.010
    front_back_w = 0.015
    front_back_d = 0.004
    for prefix, hole_x in (("left", LEFT_HOLE_X), ("right", RIGHT_HOLE_X)):
        body.visual(
            Box((front_back_w, front_back_d, collar_h)),
            origin=Origin(xyz=(hole_x, -0.007, collar_z)),
            material=brass,
            name=f"{prefix}_socket_front",
        )
        body.visual(
            Box((front_back_w, front_back_d, collar_h)),
            origin=Origin(xyz=(hole_x, 0.007, collar_z)),
            material=brass,
            name=f"{prefix}_socket_rear",
        )
        body.visual(
            Box((side_wall_w, side_wall_d, collar_h)),
            origin=Origin(xyz=(hole_x - 0.006, 0.0, collar_z)),
            material=brass,
            name=f"{prefix}_socket_inner_side",
        )
        body.visual(
            Box((side_wall_w, side_wall_d, collar_h)),
            origin=Origin(xyz=(hole_x + 0.006, 0.0, collar_z)),
            material=brass,
            name=f"{prefix}_socket_outer_side",
        )

    body.visual(
        Box((0.010, front_back_d, collar_h)),
        origin=Origin(xyz=(0.0, -0.007, collar_z)),
        material=brass_dark,
        name="center_front_bridge",
    )
    body.visual(
        Box((0.010, front_back_d, collar_h)),
        origin=Origin(xyz=(0.0, 0.007, collar_z)),
        material=brass_dark,
        name="center_rear_bridge",
    )
    body.visual(
        Box((0.0085, BODY_DEPTH, collar_h)),
        origin=Origin(xyz=(-0.01825, 0.0, collar_z)),
        material=brass_dark,
        name="left_outer_cheek",
    )
    body.visual(
        Box((0.0085, BODY_DEPTH, collar_h)),
        origin=Origin(xyz=(0.01825, 0.0, collar_z)),
        material=brass_dark,
        name="right_outer_cheek",
    )
    body.visual(
        Box((0.002, 0.0065, 0.010)),
        origin=Origin(xyz=(LEFT_HOLE_X - 0.004, 0.0, 0.0325)),
        material=brass_dark,
        name="left_pivot_liner",
    )

    body.visual(
        Cylinder(radius=0.0065, length=0.0024),
        origin=Origin(
            xyz=(0.0, BODY_FRONT_Y - 0.0012, 0.0115),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=brass_dark,
        name="key_plug",
    )
    body.visual(
        Box((0.006, 0.0015, 0.003)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - 0.0016, 0.0107)),
        material=keyway_dark,
        name="keyway_slot_top",
    )
    body.visual(
        Box((0.0022, 0.0015, 0.004)),
        origin=Origin(xyz=(0.0, BODY_FRONT_Y - 0.0016, 0.0088)),
        material=keyway_dark,
        name="keyway_slot_stem",
    )
    body.inertial = Inertial.from_geometry(
        Box((BODY_WIDTH, BODY_DEPTH, BODY_HEIGHT)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT * 0.5)),
    )

    shackle = model.part("shackle")
    shackle_path = [
        (0.000, 0.000, -0.0100),
        (0.000, 0.000, 0.0150),
        (0.003, 0.000, 0.0220),
        (0.010, 0.000, 0.0260),
        (0.017, 0.000, 0.0220),
        (0.020, 0.000, 0.0150),
        (0.020, 0.000, -0.0100),
    ]
    shackle.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                shackle_path,
                radius=0.003,
                samples_per_segment=18,
                radial_segments=20,
                cap_ends=True,
            ),
            "padlock_shackle",
        ),
        material=shackle_steel,
        name="shackle_rod",
    )
    shackle.inertial = Inertial.from_geometry(
        Box((0.026, 0.008, 0.038)),
        mass=0.08,
        origin=Origin(xyz=(0.010, 0.0, 0.008)),
    )

    release_button = model.part("release_button")
    release_button.visual(
        _rounded_block_mesh(0.0042, 0.0046, 0.0070, 0.0011, "padlock_release_button_cap"),
        origin=Origin(xyz=(-0.0016, 0.0, 0.0)),
        material=brass_dark,
        name="button_cap",
    )
    release_button.visual(
        Box((0.0060, 0.0050, 0.0038)),
        origin=Origin(xyz=(-0.0030, 0.0, 0.0)),
        material=brass_dark,
        name="button_stem",
    )
    release_button.inertial = Inertial.from_geometry(
        Box((0.007, 0.007, 0.006)),
        mass=0.01,
        origin=Origin(xyz=(-0.0030, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(LEFT_HOLE_X, HOLE_Y, BODY_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "body_to_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=release_button,
        origin=Origin(xyz=(BUTTON_OUTER_X, BUTTON_Y, BUTTON_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.02,
            lower=0.0,
            upper=BUTTON_TRAVEL,
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
    shackle = object_model.get_part("shackle")
    release_button = object_model.get_part("release_button")
    shackle_joint = object_model.get_articulation("body_to_shackle")
    button_joint = object_model.get_articulation("body_to_button")

    ctx.expect_overlap(
        release_button,
        body,
        axes="yz",
        min_overlap=0.004,
        name="release button stays aligned with the side pocket",
    )
    ctx.expect_contact(
        shackle,
        body,
        elem_a="shackle_rod",
        elem_b="left_pivot_liner",
        contact_tol=0.0002,
        name="retained leg remains seated in the pivot liner",
    )

    closed_shackle_aabb = ctx.part_world_aabb(shackle)
    closed_button_pos = ctx.part_world_position(release_button)
    ctx.check(
        "closed shackle lies in the lock plane",
        closed_shackle_aabb is not None
        and (closed_shackle_aabb[1][1] - closed_shackle_aabb[0][1]) < 0.0085,
        details=f"closed_shackle_aabb={closed_shackle_aabb}",
    )

    with ctx.pose({button_joint: BUTTON_TRAVEL}):
        pressed_button_pos = ctx.part_world_position(release_button)
        ctx.check(
            "release button presses into the body",
            closed_button_pos is not None
            and pressed_button_pos is not None
            and pressed_button_pos[0] < closed_button_pos[0] - 0.001,
            details=f"closed={closed_button_pos}, pressed={pressed_button_pos}",
        )
        ctx.expect_overlap(
            release_button,
            body,
            axes="yz",
            min_overlap=0.004,
            name="pressed button remains guided by the side wall pocket",
        )

    with ctx.pose({shackle_joint: math.radians(72.0)}):
        ctx.expect_contact(
            shackle,
            body,
            elem_a="shackle_rod",
            elem_b="left_pivot_liner",
            contact_tol=0.0002,
            name="opened shackle still turns on the retained pivot liner",
        )
        open_shackle_aabb = ctx.part_world_aabb(shackle)
        ctx.check(
            "shackle swings sideways on the retained leg",
            open_shackle_aabb is not None
            and (open_shackle_aabb[1][1] - open_shackle_aabb[0][1]) > 0.020,
            details=f"open_shackle_aabb={open_shackle_aabb}",
        )
        ctx.check(
            "open shackle clears forward of the lock center plane",
            open_shackle_aabb is not None and open_shackle_aabb[1][1] > 0.016,
            details=f"open_shackle_aabb={open_shackle_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
