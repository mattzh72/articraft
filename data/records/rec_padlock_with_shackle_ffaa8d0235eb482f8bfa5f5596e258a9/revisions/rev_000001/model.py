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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


def _aabb_center(aabb):
    if aabb is None:
        return None
    (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
    return (
        (min_x + max_x) * 0.5,
        (min_y + max_y) * 0.5,
        (min_z + max_z) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="shutter_padlock")

    body_w = 0.090
    body_t = 0.036
    body_h = 0.058
    plate_t = 0.006
    wall_w = 0.010
    shackle_r = 0.0045

    dark_steel = model.material("dark_steel", rgba=(0.19, 0.20, 0.22, 1.0))
    shackle_steel = model.material("shackle_steel", rgba=(0.74, 0.76, 0.79, 1.0))
    brass = model.material("brass", rgba=(0.74, 0.60, 0.23, 1.0))

    plate_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(body_w, body_h, radius=0.010, corner_segments=8),
            plate_t,
            cap=True,
            center=True,
            closed=True,
        ),
        "shutter_padlock_body_plate",
    )
    shackle_mesh = mesh_from_geometry(
        wire_from_points(
            [
                (0.000, 0.000, 0.000),
                (0.000, 0.000, -0.034),
                (-0.050, 0.000, -0.034),
                (-0.050, 0.000, -0.008),
            ],
            radius=shackle_r,
            closed_path=False,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.016,
            corner_segments=10,
            radial_segments=18,
        ),
        "shutter_padlock_shackle",
    )

    body = model.part("body")
    body.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, body_t * 0.5 - plate_t * 0.5, body_h * 0.5), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="front_plate",
    )
    body.visual(
        plate_mesh,
        origin=Origin(xyz=(0.0, -body_t * 0.5 + plate_t * 0.5, body_h * 0.5), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_plate",
    )
    body.visual(
        Box((wall_w, body_t, 0.044)),
        origin=Origin(xyz=(body_w * 0.5 - wall_w * 0.5, 0.0, 0.022)),
        material=dark_steel,
        name="right_wall",
    )
    body.visual(
        Box((wall_w, body_t, 0.044)),
        origin=Origin(xyz=(-body_w * 0.5 + wall_w * 0.5, 0.0, 0.022)),
        material=dark_steel,
        name="left_wall",
    )
    body.visual(
        Box((0.066, body_t, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
        material=dark_steel,
        name="top_bridge",
    )
    body.visual(
        Box((0.032, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.013, 0.012)),
        material=dark_steel,
        name="front_chin",
    )
    body.visual(
        Box((0.032, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.013, 0.012)),
        material=dark_steel,
        name="rear_chin",
    )
    body.visual(
        Box((0.010, 0.007, 0.020)),
        origin=Origin(xyz=(0.020, 0.0085, 0.026)),
        material=dark_steel,
        name="front_pivot_cheek",
    )
    body.visual(
        Box((0.010, 0.007, 0.020)),
        origin=Origin(xyz=(0.020, -0.0085, 0.026)),
        material=dark_steel,
        name="rear_pivot_cheek",
    )
    body.visual(
        Cylinder(radius=0.0115, length=0.004),
        origin=Origin(xyz=(0.0, 0.0165, 0.022), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="key_cylinder_bezel",
    )
    body.visual(
        Cylinder(radius=0.0088, length=0.005),
        origin=Origin(xyz=(0.0, 0.0155, 0.022), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="key_cylinder",
    )
    body.visual(
        Box((0.003, 0.0015, 0.010)),
        origin=Origin(xyz=(0.0, 0.0185, 0.021)),
        material=dark_steel,
        name="keyway_slot",
    )
    body.visual(
        Box((0.008, 0.0015, 0.003)),
        origin=Origin(xyz=(0.0, 0.0185, 0.0165)),
        material=dark_steel,
        name="keyway_ward",
    )
    body.visual(
        Cylinder(radius=0.0042, length=0.002),
        origin=Origin(xyz=(0.018, 0.019, 0.022), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="cap_pivot_stud",
    )
    body.inertial = Inertial.from_geometry(
        Box((body_w, body_t, body_h)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, body_h * 0.5)),
    )

    shackle = model.part("shackle")
    shackle.visual(
        shackle_mesh,
        material=shackle_steel,
        name="shackle_bar",
    )
    shackle.visual(
        Cylinder(radius=0.0043, length=0.010),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shackle_steel,
        name="pivot_journal",
    )
    shackle.visual(
        Sphere(radius=shackle_r * 0.98),
        origin=Origin(xyz=(-0.050, 0.0, -0.008)),
        material=shackle_steel,
        name="free_tip_marker",
    )
    shackle.inertial = Inertial.from_geometry(
        Box((0.059, shackle_r * 2.2, 0.043)),
        mass=0.45,
        origin=Origin(xyz=(-0.025, 0.0, -0.017)),
    )

    key_cap = model.part("key_cap")
    key_cap.visual(
        Cylinder(radius=0.0052, length=0.003),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="cap_knuckle",
    )
    key_cap.visual(
        Cylinder(radius=0.014, length=0.004),
        origin=Origin(xyz=(-0.018, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="cap_disc",
    )
    key_cap.visual(
        Box((0.006, 0.004, 0.008)),
        origin=Origin(xyz=(-0.030, 0.0, -0.002)),
        material=dark_steel,
        name="cap_lip",
    )
    key_cap.inertial = Inertial.from_geometry(
        Cylinder(radius=0.014, length=0.006),
        mass=0.08,
        origin=Origin(xyz=(-0.018, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "body_to_shackle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=shackle,
        origin=Origin(xyz=(0.020, 0.0, 0.034)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=2.0,
            lower=0.0,
            upper=1.15,
        ),
    )
    model.articulation(
        "body_to_key_cap",
        ArticulationType.REVOLUTE,
        parent=body,
        child=key_cap,
        origin=Origin(xyz=(0.018, 0.0215, 0.022)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=4.0,
            lower=0.0,
            upper=1.8,
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
    key_cap = object_model.get_part("key_cap")
    shackle_joint = object_model.get_articulation("body_to_shackle")
    cap_joint = object_model.get_articulation("body_to_key_cap")

    ctx.expect_gap(
        key_cap,
        body,
        axis="y",
        positive_elem="cap_disc",
        negative_elem="front_plate",
        min_gap=0.001,
        max_gap=0.006,
        name="key cap sits just proud of the front plate",
    )

    with ctx.pose({cap_joint: 0.0}):
        ctx.expect_overlap(
            key_cap,
            body,
            axes="xz",
            elem_a="cap_disc",
            elem_b="key_cylinder_bezel",
            min_overlap=0.020,
            name="closed cap covers the key cylinder",
        )
        closed_cap = _aabb_center(ctx.part_element_world_aabb(key_cap, elem="cap_disc"))
        bezel_center = _aabb_center(ctx.part_element_world_aabb(body, elem="key_cylinder_bezel"))

    with ctx.pose({cap_joint: 1.45}):
        open_cap = _aabb_center(ctx.part_element_world_aabb(key_cap, elem="cap_disc"))

    ctx.check(
        "key cap rotates clear of the key cylinder",
        closed_cap is not None
        and open_cap is not None
        and bezel_center is not None
        and open_cap[2] > closed_cap[2] + 0.010
        and abs(open_cap[0] - bezel_center[0]) > 0.010,
        details=f"closed_cap={closed_cap}, open_cap={open_cap}, bezel={bezel_center}",
    )

    with ctx.pose({shackle_joint: 0.0}):
        closed_tip = _aabb_center(ctx.part_element_world_aabb(shackle, elem="free_tip_marker"))

    with ctx.pose({shackle_joint: 1.05}):
        open_tip = _aabb_center(ctx.part_element_world_aabb(shackle, elem="free_tip_marker"))

    ctx.check(
        "closed shackle tip stays tucked in the left slot",
        closed_tip is not None and closed_tip[0] < -0.020 and closed_tip[2] > 0.020,
        details=f"closed_tip={closed_tip}",
    )
    ctx.check(
        "shackle swings down and away on the retained side",
        closed_tip is not None
        and open_tip is not None
        and open_tip[2] < closed_tip[2] - 0.020
        and open_tip[0] > closed_tip[0] + 0.015,
        details=f"closed_tip={closed_tip}, open_tip={open_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
