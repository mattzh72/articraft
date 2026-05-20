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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _rot_y(angle: float, x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, angle, 0.0))


def _aabb_center(aabb):
    if aabb is None:
        return None
    mn, mx = aabb
    return tuple((mn[i] + mx[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="overshot_wheel_pit")

    stone = model.material("stone", rgba=(0.62, 0.60, 0.56, 1.0))
    stone_dark = model.material("stone_dark", rgba=(0.46, 0.45, 0.42, 1.0))
    damp_stone = model.material("damp_stone", rgba=(0.39, 0.41, 0.40, 1.0))
    weathered_wood = model.material("weathered_wood", rgba=(0.44, 0.31, 0.20, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.21, 0.22, 0.24, 1.0))
    iron = model.material("iron", rgba=(0.43, 0.45, 0.47, 1.0))

    wall_length = 3.10
    wall_height = 2.70
    wall_thickness = 0.34
    wall_center_y = 0.50
    floor_width = 1.34
    floor_thickness = 0.16
    opening_width = 0.50
    opening_height = 0.60
    opening_bottom = 0.22
    axle_height = 1.45

    pit = model.part("pit")
    pit.visual(
        Box((wall_length, floor_width, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness * 0.5)),
        material=damp_stone,
        name="pit_floor",
    )

    jamb_size_x = (wall_length - opening_width) * 0.5
    jamb_center_offset = (opening_width + jamb_size_x) * 0.5

    pit.visual(
        Box((wall_length, wall_thickness, opening_bottom)),
        origin=Origin(xyz=(0.0, -wall_center_y, opening_bottom * 0.5)),
        material=stone_dark,
        name="left_wall_lower",
    )
    pit.visual(
        Box((wall_length, wall_thickness, wall_height - (opening_bottom + opening_height))),
        origin=Origin(
            xyz=(
                0.0,
                -wall_center_y,
                opening_bottom + opening_height + (wall_height - (opening_bottom + opening_height)) * 0.5,
            )
        ),
        material=stone,
        name="left_wall_upper",
    )
    pit.visual(
        Box((jamb_size_x, wall_thickness, opening_height)),
        origin=Origin(
            xyz=(-jamb_center_offset, -wall_center_y, opening_bottom + opening_height * 0.5)
        ),
        material=stone,
        name="left_wall_jamb_a",
    )
    pit.visual(
        Box((jamb_size_x, wall_thickness, opening_height)),
        origin=Origin(
            xyz=(jamb_center_offset, -wall_center_y, opening_bottom + opening_height * 0.5)
        ),
        material=stone,
        name="left_wall_jamb_b",
    )
    pit.visual(
        Box((wall_length, wall_thickness, wall_height)),
        origin=Origin(xyz=(0.0, wall_center_y, wall_height * 0.5)),
        material=stone,
        name="right_wall",
    )
    pit.visual(
        Box((wall_length + 0.08, wall_thickness + 0.04, 0.08)),
        origin=Origin(xyz=(0.0, -wall_center_y, wall_height + 0.04)),
        material=stone_dark,
        name="left_coping",
    )
    pit.visual(
        Box((wall_length + 0.08, wall_thickness + 0.04, 0.08)),
        origin=Origin(xyz=(0.0, wall_center_y, wall_height + 0.04)),
        material=stone_dark,
        name="right_coping",
    )
    pit.visual(
        Box((0.34, 0.11, 0.26)),
        origin=Origin(xyz=(0.0, -0.275, axle_height)),
        material=dark_iron,
        name="left_bearing_block",
    )
    pit.visual(
        Box((0.34, 0.11, 0.26)),
        origin=Origin(xyz=(0.0, 0.275, axle_height)),
        material=dark_iron,
        name="right_bearing_block",
    )
    pit.visual(
        Box((0.18, 0.16, 0.05)),
        origin=Origin(xyz=(0.0, -0.275, axle_height + 0.125)),
        material=iron,
        name="left_bearing_cap",
    )
    pit.visual(
        Box((0.18, 0.16, 0.05)),
        origin=Origin(xyz=(0.0, 0.275, axle_height + 0.125)),
        material=iron,
        name="right_bearing_cap",
    )
    pit.inertial = Inertial.from_geometry(
        Box((wall_length, floor_width, wall_height + 0.12)),
        mass=8200.0,
        origin=Origin(xyz=(0.0, 0.0, (wall_height + 0.12) * 0.5)),
    )

    wheel = model.part("wheel")
    wheel.visual(
        Cylinder(radius=0.07, length=0.44),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dark_iron,
        name="axle",
    )
    wheel.visual(
        Cylinder(radius=0.20, length=0.24),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=dark_iron,
        name="hub_drum",
    )
    wheel.visual(
        Cylinder(radius=0.28, length=0.03),
        origin=Origin(xyz=(0.0, -0.125, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=iron,
        name="left_hub_flange",
    )
    wheel.visual(
        Cylinder(radius=0.28, length=0.03),
        origin=Origin(xyz=(0.0, 0.125, 0.0), rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=iron,
        name="right_hub_flange",
    )

    crown_radius = 1.14
    spoke_length = 0.92
    spoke_center_radius = 0.68
    spoke_width = 0.12
    paddle_center_radius = 1.06
    paddle_width = 0.40
    paddle_radial = 0.16
    paddle_tangent = 0.20
    bucket_count = 16

    crown_mesh = mesh_from_geometry(
        TorusGeometry(radius=crown_radius, tube=0.04, radial_segments=18, tubular_segments=64).rotate_x(
            math.pi * 0.5
        ),
        "overshot_wheel_crown",
    )
    wheel.visual(
        crown_mesh,
        origin=Origin(xyz=(0.0, -0.18, 0.0)),
        material=dark_iron,
        name="left_crown",
    )
    wheel.visual(
        crown_mesh,
        origin=Origin(xyz=(0.0, 0.18, 0.0)),
        material=dark_iron,
        name="right_crown",
    )

    for index in range(bucket_count):
        angle = math.pi * 0.5 + (2.0 * math.pi * index) / bucket_count
        x = spoke_center_radius * math.cos(angle)
        z = spoke_center_radius * math.sin(angle)
        wheel.visual(
            Box((spoke_length, spoke_width, 0.06)),
            origin=_rot_y(angle, x, -0.125, z),
            material=weathered_wood,
            name="left_spoke_00" if index == 0 else None,
        )
        wheel.visual(
            Box((spoke_length, spoke_width, 0.06)),
            origin=_rot_y(angle, x, 0.125, z),
            material=weathered_wood,
            name="right_spoke_00" if index == 0 else None,
        )

        paddle_angle = angle - 0.22
        px = paddle_center_radius * math.cos(angle)
        pz = paddle_center_radius * math.sin(angle)
        wheel.visual(
            Box((paddle_radial, paddle_width, paddle_tangent)),
            origin=_rot_y(paddle_angle, px, 0.0, pz),
            material=weathered_wood,
            name="bucket_00" if index == 0 else None,
        )

    wheel.inertial = Inertial.from_geometry(
        Box((2.36, 0.44, 2.36)),
        mass=320.0,
        origin=Origin(),
    )

    gate = model.part("access_gate")
    gate.visual(
        Box((0.44, 0.05, 0.56)),
        origin=Origin(xyz=(0.22, 0.0, 0.28)),
        material=weathered_wood,
        name="gate_leaf",
    )
    gate.visual(
        Box((0.03, 0.06, 0.56)),
        origin=Origin(xyz=(0.015, -0.005, 0.28)),
        material=weathered_wood,
        name="hinge_stile",
    )
    gate.visual(
        Box((0.44, 0.018, 0.05)),
        origin=Origin(xyz=(0.22, -0.034, 0.14), rpy=(0.0, -0.62, 0.0)),
        material=dark_iron,
        name="lower_strap",
    )
    gate.visual(
        Box((0.44, 0.018, 0.05)),
        origin=Origin(xyz=(0.22, -0.034, 0.42), rpy=(0.0, -0.62, 0.0)),
        material=dark_iron,
        name="upper_strap",
    )
    gate.visual(
        Cylinder(radius=0.015, length=0.16),
        origin=Origin(xyz=(0.0, -0.048, 0.10)),
        material=dark_iron,
        name="hinge_barrel_lower",
    )
    gate.visual(
        Cylinder(radius=0.015, length=0.16),
        origin=Origin(xyz=(0.0, -0.048, 0.46)),
        material=dark_iron,
        name="hinge_barrel_upper",
    )
    gate.inertial = Inertial.from_geometry(
        Box((0.44, 0.05, 0.56)),
        mass=34.0,
        origin=Origin(xyz=(0.22, 0.0, 0.28)),
    )

    model.articulation(
        "pit_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=pit,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, axle_height)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.6),
    )
    model.articulation(
        "pit_to_access_gate",
        ArticulationType.REVOLUTE,
        parent=pit,
        child=gate,
        origin=Origin(xyz=(-0.235, -wall_center_y, opening_bottom + 0.02)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.5,
            lower=0.0,
            upper=1.45,
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

    pit = object_model.get_part("pit")
    wheel = object_model.get_part("wheel")
    gate = object_model.get_part("access_gate")
    wheel_spin = object_model.get_articulation("pit_to_wheel")
    gate_hinge = object_model.get_articulation("pit_to_access_gate")

    ctx.expect_contact(
        wheel,
        pit,
        elem_a="axle",
        elem_b="left_bearing_block",
        contact_tol=0.0015,
        name="axle seats against left bearing block",
    )
    ctx.expect_contact(
        wheel,
        pit,
        elem_a="axle",
        elem_b="right_bearing_block",
        contact_tol=0.0015,
        name="axle seats against right bearing block",
    )
    ctx.expect_origin_gap(
        wheel,
        gate,
        axis="z",
        min_gap=0.85,
        name="access gate sits well below the wheel axle line",
    )

    bucket_rest = ctx.part_element_world_aabb(wheel, elem="bucket_00")
    with ctx.pose({wheel_spin: math.pi * 0.5}):
        bucket_turned = ctx.part_element_world_aabb(wheel, elem="bucket_00")
    bucket_rest_center = _aabb_center(bucket_rest)
    bucket_turned_center = _aabb_center(bucket_turned)
    ctx.check(
        "wheel continuous joint actually rotates the crown buckets",
        bucket_rest_center is not None
        and bucket_turned_center is not None
        and bucket_turned_center[0] > bucket_rest_center[0] + 0.65
        and abs(bucket_turned_center[2]) < bucket_rest_center[2] - 0.65,
        details=f"rest={bucket_rest_center}, turned={bucket_turned_center}",
    )

    gate_closed = ctx.part_world_aabb(gate)
    with ctx.pose({gate_hinge: 1.10}):
        gate_open = ctx.part_world_aabb(gate)
    ctx.check(
        "access gate swings outward from the wall opening",
        gate_closed is not None
        and gate_open is not None
        and gate_open[0][1] < gate_closed[0][1] - 0.18,
        details=f"closed={gate_closed}, open={gate_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
