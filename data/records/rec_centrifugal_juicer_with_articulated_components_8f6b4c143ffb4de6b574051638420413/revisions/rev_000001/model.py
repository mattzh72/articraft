from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _xy_section(width: float, depth: float, radius: float, z: float) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, y in rounded_rect_profile(width, depth, radius)]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_juicer")

    body_white = model.material("body_white", rgba=(0.93, 0.93, 0.91, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.18, 1.0))
    stainless = model.material("stainless", rgba=(0.72, 0.74, 0.76, 1.0))
    clear_smoke = model.material("clear_smoke", rgba=(0.76, 0.82, 0.86, 0.35))
    clear_plastic = model.material("clear_plastic", rgba=(0.90, 0.94, 0.98, 0.30))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_cavity = model.material("dark_cavity", rgba=(0.08, 0.09, 0.10, 1.0))

    housing_geom = section_loft(
        [
            _xy_section(0.252, 0.192, 0.030, 0.000),
            _xy_section(0.282, 0.224, 0.040, 0.034),
            _xy_section(0.266, 0.208, 0.038, 0.096),
            _xy_section(0.228, 0.182, 0.030, 0.152),
        ]
    )
    housing_mesh = mesh_from_geometry(housing_geom, "housing_shell")

    top_collar_geom = LatheGeometry.from_shell_profiles(
        [
            (0.118, 0.000),
            (0.118, 0.010),
            (0.111, 0.019),
            (0.101, 0.025),
        ],
        [
            (0.090, 0.000),
            (0.090, 0.018),
            (0.086, 0.025),
        ],
        segments=56,
    )
    top_collar_mesh = mesh_from_geometry(top_collar_geom, "top_collar")

    lid_dome_geom = LatheGeometry.from_shell_profiles(
        [
            (0.115, 0.000),
            (0.111, 0.018),
            (0.096, 0.055),
            (0.055, 0.090),
            (0.045, 0.095),
        ],
        [
            (0.107, 0.003),
            (0.103, 0.020),
            (0.089, 0.053),
            (0.050, 0.087),
            (0.037, 0.092),
        ],
        segments=56,
    )
    lid_dome_mesh = mesh_from_geometry(lid_dome_geom, "lid_dome")

    chute_geom = LatheGeometry.from_shell_profiles(
        [(0.045, 0.000), (0.045, 0.068)],
        [(0.037, 0.002), (0.037, 0.066)],
        segments=40,
    )
    chute_mesh = mesh_from_geometry(chute_geom, "lid_chute")

    basket_shell_geom = LatheGeometry.from_shell_profiles(
        [
            (0.060, 0.000),
            (0.075, 0.005),
            (0.080, 0.012),
            (0.080, 0.060),
            (0.074, 0.076),
        ],
        [
            (0.050, 0.004),
            (0.069, 0.011),
            (0.073, 0.057),
            (0.068, 0.072),
        ],
        segments=48,
    )
    basket_shell_mesh = mesh_from_geometry(basket_shell_geom, "basket_shell")

    cone_geom = ConeGeometry(radius=0.029, height=0.042).translate(0.0, 0.0, 0.050)
    cone_geom.merge(CylinderGeometry(radius=0.010, height=0.048).translate(0.0, 0.0, 0.024))
    basket_cone_mesh = mesh_from_geometry(cone_geom, "basket_cone")

    body = model.part("body")
    body.visual(housing_mesh, material=body_white, name="housing")
    body.visual(
        top_collar_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.150)),
        material=stainless,
        name="top_collar",
    )
    body.visual(
        Cylinder(radius=0.008, length=0.016),
        origin=Origin(xyz=(0.000, 0.000, 0.148)),
        material=stainless,
        name="drive_spindle",
    )
    for x in (-0.092, 0.092):
        for y in (-0.072, 0.072):
            body.visual(
                Box((0.034, 0.026, 0.006)),
                origin=Origin(xyz=(x, y, -0.003)),
                material=charcoal,
                name=f"foot_{int((x > 0))}_{int((y > 0))}",
            )
    for y in (-0.050, 0.050):
        body.visual(
            Box((0.022, 0.022, 0.024)),
            origin=Origin(xyz=(-0.122, y, 0.165)),
            material=charcoal,
            name=f"rear_hinge_block_{int(y > 0)}",
        )
    body.visual(
        Box((0.102, 0.008, 0.112)),
        origin=Origin(xyz=(0.008, 0.099, 0.096)),
        material=dark_cavity,
        name="service_cavity",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.282, 0.224, 0.177)),
        mass=4.8,
        origin=Origin(xyz=(0.000, 0.000, 0.088)),
    )

    basket = model.part("basket")
    basket.visual(
        basket_shell_mesh,
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=stainless,
        name="basket_shell",
    )
    basket.visual(
        Cylinder(radius=0.055, length=0.004),
        origin=Origin(xyz=(0.000, 0.000, 0.002)),
        material=stainless,
        name="basket_floor",
    )
    basket.visual(
        basket_cone_mesh,
        material=stainless,
        name="basket_cone",
    )
    basket.inertial = Inertial.from_geometry(
        Cylinder(radius=0.080, length=0.080),
        mass=0.40,
        origin=Origin(xyz=(0.000, 0.000, 0.040)),
    )

    lid = model.part("lid")
    lid.visual(
        lid_dome_mesh,
        origin=Origin(xyz=(0.115, 0.000, 0.000)),
        material=clear_smoke,
        name="lid_dome",
    )
    lid.visual(
        chute_mesh,
        origin=Origin(xyz=(0.115, 0.000, 0.095)),
        material=clear_plastic,
        name="chute_shell",
    )
    lid.visual(
        Box((0.032, 0.136, 0.010)),
        origin=Origin(xyz=(0.016, 0.000, 0.008)),
        material=charcoal,
        name="hinge_spine",
    )
    for y in (-0.042, 0.042):
        lid.visual(
            Cylinder(radius=0.007, length=0.036),
            origin=Origin(xyz=(0.000, y, 0.010), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name=f"hinge_barrel_{int(y > 0)}",
        )
    lid.inertial = Inertial.from_geometry(
        Box((0.235, 0.230, 0.165)),
        mass=0.65,
        origin=Origin(xyz=(0.115, 0.000, 0.082)),
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.032, length=0.062),
        origin=Origin(xyz=(0.000, 0.000, -0.031)),
        material=clear_plastic,
        name="pusher_stem",
    )
    pusher.visual(
        Cylinder(radius=0.050, length=0.022),
        origin=Origin(xyz=(0.000, 0.000, 0.011)),
        material=clear_plastic,
        name="pusher_cap",
    )
    pusher.inertial = Inertial.from_geometry(
        Cylinder(radius=0.050, length=0.084),
        mass=0.18,
        origin=Origin(xyz=(0.000, 0.000, -0.020)),
    )

    door = model.part("service_door")
    door.visual(
        Box((0.092, 0.008, 0.108)),
        origin=Origin(xyz=(-0.046, 0.004, 0.000)),
        material=body_white,
        name="door_panel",
    )
    door.visual(
        Box((0.014, 0.012, 0.036)),
        origin=Origin(xyz=(-0.088, 0.010, -0.004)),
        material=charcoal,
        name="door_latch",
    )
    for z in (-0.034, 0.034):
        door.visual(
            Cylinder(radius=0.005, length=0.028),
            origin=Origin(xyz=(0.000, 0.004, z), rpy=(0.0, 0.0, 0.0)),
            material=charcoal,
            name=f"door_hinge_barrel_{int(z > 0)}",
        )
    door.inertial = Inertial.from_geometry(
        Box((0.092, 0.016, 0.108)),
        mass=0.16,
        origin=Origin(xyz=(-0.046, 0.008, 0.000)),
    )

    knob = model.part("control_knob")
    knob.visual(
        Cylinder(radius=0.007, length=0.008),
        origin=Origin(xyz=(0.000, 0.004, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="knob_shaft",
    )
    knob.visual(
        Cylinder(radius=0.023, length=0.016),
        origin=Origin(xyz=(0.000, 0.016, 0.000), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="knob_body",
    )
    knob.visual(
        Box((0.004, 0.014, 0.018)),
        origin=Origin(xyz=(0.000, 0.024, 0.016)),
        material=stainless,
        name="knob_pointer",
    )
    knob.inertial = Inertial.from_geometry(
        Box((0.046, 0.032, 0.046)),
        mass=0.08,
        origin=Origin(xyz=(0.000, 0.016, 0.000)),
    )

    model.articulation(
        "body_to_basket",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=basket,
        origin=Origin(xyz=(0.000, 0.000, 0.156)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=35.0),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(-0.115, 0.000, 0.173)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=0.0,
            upper=1.15,
        ),
    )

    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.115, 0.000, 0.163)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.20,
            lower=0.0,
            upper=0.038,
        ),
    )

    model.articulation(
        "body_to_service_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.056, 0.109, 0.096)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.5,
            lower=0.0,
            upper=1.25,
        ),
    )

    model.articulation(
        "body_to_control_knob",
        ArticulationType.REVOLUTE,
        parent=body,
        child=knob,
        origin=Origin(xyz=(0.004, 0.110, 0.040)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.8,
            velocity=2.5,
            lower=-2.4,
            upper=2.4,
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
    lid = object_model.get_part("lid")
    pusher = object_model.get_part("pusher")
    basket = object_model.get_part("basket")
    door = object_model.get_part("service_door")
    knob = object_model.get_part("control_knob")

    lid_hinge = object_model.get_articulation("body_to_lid")
    pusher_slide = object_model.get_articulation("lid_to_pusher")
    door_hinge = object_model.get_articulation("body_to_service_door")
    knob_joint = object_model.get_articulation("body_to_control_knob")

    ctx.expect_within(
        pusher,
        lid,
        axes="xy",
        inner_elem="pusher_stem",
        outer_elem="chute_shell",
        margin=0.006,
        name="pusher stays centered in chute",
    )
    ctx.expect_within(
        basket,
        body,
        axes="xy",
        inner_elem="basket_shell",
        outer_elem="top_collar",
        margin=0.012,
        name="basket stays inside top collar",
    )
    ctx.expect_overlap(
        lid,
        body,
        axes="xy",
        elem_a="lid_dome",
        elem_b="top_collar",
        min_overlap=0.160,
        name="lid covers juicing chamber",
    )
    ctx.expect_origin_gap(
        door,
        knob,
        axis="z",
        min_gap=0.040,
        name="control knob sits below service door",
    )

    closed_lid_aabb = ctx.part_world_aabb(lid)
    pressed_pusher_pos = None
    opened_lid_aabb = None
    opened_door_aabb = None
    rotated_knob_pos = None

    with ctx.pose({pusher_slide: 0.038}):
        ctx.expect_within(
            pusher,
            lid,
            axes="xy",
            inner_elem="pusher_stem",
            outer_elem="chute_shell",
            margin=0.006,
            name="pressed pusher stays centered in chute",
        )
        pressed_pusher_pos = ctx.part_world_position(pusher)

    with ctx.pose({lid_hinge: 1.0}):
        opened_lid_aabb = ctx.part_world_aabb(lid)

    with ctx.pose({door_hinge: 1.0}):
        opened_door_aabb = ctx.part_world_aabb(door)

    with ctx.pose({knob_joint: 1.6}):
        rotated_knob_pos = ctx.part_world_position(knob)

    rest_pusher_pos = ctx.part_world_position(pusher)
    closed_door_aabb = ctx.part_world_aabb(door)
    rest_knob_pos = ctx.part_world_position(knob)

    ctx.check(
        "pusher moves downward into chute",
        rest_pusher_pos is not None
        and pressed_pusher_pos is not None
        and pressed_pusher_pos[2] < rest_pusher_pos[2] - 0.030,
        details=f"rest={rest_pusher_pos}, pressed={pressed_pusher_pos}",
    )
    ctx.check(
        "lid opens upward",
        closed_lid_aabb is not None
        and opened_lid_aabb is not None
        and opened_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.080,
        details=f"closed={closed_lid_aabb}, opened={opened_lid_aabb}",
    )
    ctx.check(
        "service door swings outward",
        closed_door_aabb is not None
        and opened_door_aabb is not None
        and opened_door_aabb[1][1] > closed_door_aabb[1][1] + 0.040,
        details=f"closed={closed_door_aabb}, opened={opened_door_aabb}",
    )
    ctx.check(
        "control knob rotates in place",
        rest_knob_pos is not None
        and rotated_knob_pos is not None
        and abs(rotated_knob_pos[1] - rest_knob_pos[1]) < 0.002,
        details=f"rest={rest_knob_pos}, rotated={rotated_knob_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
