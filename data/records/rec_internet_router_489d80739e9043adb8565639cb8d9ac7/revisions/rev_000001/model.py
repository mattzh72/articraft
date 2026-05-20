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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _build_router_shell_mesh() -> MeshGeometry:
    geom = MeshGeometry()

    bottom = [
        (-0.160, -0.100, 0.012),
        (0.160, -0.100, 0.012),
        (0.170, 0.015, 0.012),
        (0.135, 0.100, 0.012),
        (-0.135, 0.100, 0.012),
        (-0.170, 0.015, 0.012),
    ]
    top = [
        (-0.138, -0.084, 0.031),
        (0.138, -0.084, 0.031),
        (0.146, 0.010, 0.026),
        (0.098, 0.078, 0.019),
        (-0.098, 0.078, 0.019),
        (-0.146, 0.010, 0.026),
    ]

    b_ids = [geom.add_vertex(*point) for point in bottom]
    t_ids = [geom.add_vertex(*point) for point in top]

    for index in range(len(bottom)):
        next_index = (index + 1) % len(bottom)
        _add_quad(geom, b_ids[index], b_ids[next_index], t_ids[next_index], t_ids[index])

    geom.add_face(b_ids[0], b_ids[2], b_ids[1])
    geom.add_face(b_ids[0], b_ids[3], b_ids[2])
    geom.add_face(b_ids[0], b_ids[4], b_ids[3])
    geom.add_face(b_ids[0], b_ids[5], b_ids[4])

    geom.add_face(t_ids[0], t_ids[1], t_ids[2])
    geom.add_face(t_ids[0], t_ids[2], t_ids[3])
    geom.add_face(t_ids[0], t_ids[3], t_ids[4])
    geom.add_face(t_ids[0], t_ids[4], t_ids[5])

    return geom


def _add_rear_antenna_visuals(part, shell_material, accent_material) -> None:
    part.visual(
        Cylinder(radius=0.0036, length=0.024),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=shell_material,
        name="hinge_barrel",
    )
    part.visual(
        Box((0.020, 0.008, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=shell_material,
        name="lower_stem",
    )
    part.visual(
        Box((0.022, 0.007, 0.132)),
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
        material=shell_material,
        name="main_blade",
    )
    part.visual(
        Box((0.016, 0.006, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.164)),
        material=shell_material,
        name="tip_cap",
    )
    part.visual(
        Box((0.005, 0.0074, 0.074)),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=accent_material,
        name="accent_strip",
    )


def _add_side_antenna_visuals(part, shell_material, accent_material) -> None:
    part.visual(
        Cylinder(radius=0.0036, length=0.024),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=shell_material,
        name="hinge_barrel",
    )
    part.visual(
        Box((0.008, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=shell_material,
        name="lower_stem",
    )
    part.visual(
        Box((0.007, 0.022, 0.132)),
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
        material=shell_material,
        name="main_blade",
    )
    part.visual(
        Box((0.006, 0.016, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.164)),
        material=shell_material,
        name="tip_cap",
    )
    part.visual(
        Box((0.0074, 0.005, 0.074)),
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        material=accent_material,
        name="accent_strip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gaming_router")

    housing_dark = model.material("housing_dark", rgba=(0.11, 0.12, 0.14, 1.0))
    housing_black = model.material("housing_black", rgba=(0.06, 0.07, 0.08, 1.0))
    antenna_black = model.material("antenna_black", rgba=(0.08, 0.09, 0.10, 1.0))
    gaming_red = model.material("gaming_red", rgba=(0.77, 0.13, 0.16, 1.0))
    knob_red = model.material("knob_red", rgba=(0.62, 0.11, 0.14, 1.0))

    housing = model.part("housing")
    housing.visual(
        Box((0.340, 0.230, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=housing_black,
        name="base_tray",
    )
    housing.visual(
        mesh_from_geometry(_build_router_shell_mesh(), "router_shell"),
        material=housing_dark,
        name="body_shell",
    )
    housing.visual(
        Box((0.122, 0.048, 0.006)),
        origin=Origin(xyz=(0.0, -0.018, 0.031)),
        material=gaming_red,
        name="rear_spine_accent",
    )
    housing.visual(
        Box((0.070, 0.012, 0.004)),
        origin=Origin(xyz=(-0.080, 0.060, 0.021)),
        material=gaming_red,
        name="left_front_accent",
    )
    housing.visual(
        Box((0.050, 0.012, 0.004)),
        origin=Origin(xyz=(0.032, 0.060, 0.021)),
        material=gaming_red,
        name="center_front_accent",
    )
    housing.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.114, 0.060, 0.020)),
        material=housing_black,
        name="corner_pod",
    )

    rear_x_positions = (-0.120, -0.040, 0.040, 0.120)
    for index, x_pos in enumerate(rear_x_positions):
        housing.visual(
            Box((0.028, 0.016, 0.010)),
            origin=Origin(xyz=(x_pos, -0.089, 0.036)),
            material=housing_black,
            name=f"rear_hinge_pedestal_{index}",
        )

    housing.visual(
        Box((0.020, 0.028, 0.010)),
        origin=Origin(xyz=(-0.155, -0.016, 0.029)),
        material=housing_black,
        name="left_hinge_pedestal",
    )
    housing.visual(
        Box((0.020, 0.028, 0.010)),
        origin=Origin(xyz=(0.155, -0.016, 0.029)),
        material=housing_black,
        name="right_hinge_pedestal",
    )

    rear_names = [
        "rear_left_outer_antenna",
        "rear_left_inner_antenna",
        "rear_right_inner_antenna",
        "rear_right_outer_antenna",
    ]
    for part_name in rear_names:
        _add_rear_antenna_visuals(model.part(part_name), antenna_black, gaming_red)

    _add_side_antenna_visuals(model.part("left_side_antenna"), antenna_black, gaming_red)
    _add_side_antenna_visuals(model.part("right_side_antenna"), antenna_black, gaming_red)

    mode_knob = model.part("mode_knob")
    mode_knob.visual(
        Cylinder(radius=0.005, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=housing_black,
        name="shaft_stub",
    )
    mode_knob.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=knob_red,
        name="knob_flange",
    )
    mode_knob.visual(
        Cylinder(radius=0.014, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=housing_black,
        name="knob_body",
    )
    mode_knob.visual(
        Box((0.004, 0.011, 0.003)),
        origin=Origin(xyz=(0.0, 0.011, 0.018)),
        material=gaming_red,
        name="indicator_tab",
    )

    for part_name, x_pos in zip(rear_names, rear_x_positions):
        model.articulation(
            f"housing_to_{part_name}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=part_name,
            origin=Origin(xyz=(x_pos, -0.089, 0.0416)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.2,
                velocity=2.2,
                lower=-0.20,
                upper=1.35,
            ),
        )

    model.articulation(
        "housing_to_left_side_antenna",
        ArticulationType.REVOLUTE,
        parent=housing,
        child="left_side_antenna",
        origin=Origin(xyz=(-0.155, -0.016, 0.0346)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.2,
            lower=-0.20,
            upper=1.35,
        ),
    )
    model.articulation(
        "housing_to_right_side_antenna",
        ArticulationType.REVOLUTE,
        parent=housing,
        child="right_side_antenna",
        origin=Origin(xyz=(0.155, -0.016, 0.0346)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.2,
            velocity=2.2,
            lower=-0.20,
            upper=1.35,
        ),
    )
    model.articulation(
        "housing_to_mode_knob",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=mode_knob,
        origin=Origin(xyz=(0.114, 0.060, 0.026)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=-1.15,
            upper=1.15,
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

    housing = object_model.get_part("housing")
    rear_antenna_names = [
        "rear_left_outer_antenna",
        "rear_left_inner_antenna",
        "rear_right_inner_antenna",
        "rear_right_outer_antenna",
    ]
    side_antenna_names = ["left_side_antenna", "right_side_antenna"]

    for part_name in ["housing", *rear_antenna_names, *side_antenna_names, "mode_knob"]:
        part = object_model.get_part(part_name)
        ctx.check(f"{part_name} is present", part is not None)

    for joint_name in [
        *(f"housing_to_{name}" for name in rear_antenna_names),
        "housing_to_left_side_antenna",
        "housing_to_right_side_antenna",
        "housing_to_mode_knob",
    ]:
        joint = object_model.get_articulation(joint_name)
        ctx.check(f"{joint_name} articulation is present", joint is not None)

    for antenna_name in [*rear_antenna_names, *side_antenna_names]:
        antenna = object_model.get_part(antenna_name)
        ctx.expect_contact(
            antenna,
            housing,
            contact_tol=0.0015,
            name=f"{antenna_name} is mounted to the housing",
        )

    knob = object_model.get_part("mode_knob")

    def part_center(part) -> tuple[float, float, float] | None:
        aabb = ctx.part_world_aabb(part)
        if aabb is None:
            return None
        mins, maxs = aabb
        return (
            (mins[0] + maxs[0]) * 0.5,
            (mins[1] + maxs[1]) * 0.5,
            (mins[2] + maxs[2]) * 0.5,
        )

    ctx.expect_contact(
        knob,
        housing,
        contact_tol=0.0015,
        name="mode knob seats on the corner pod",
    )

    for antenna_name in rear_antenna_names:
        antenna = object_model.get_part(antenna_name)
        joint = object_model.get_articulation(f"housing_to_{antenna_name}")
        rest_pos = part_center(antenna)
        with ctx.pose({joint: 1.10}):
            folded_pos = part_center(antenna)
        ctx.check(
            f"{antenna_name} folds rearward",
            rest_pos is not None
            and folded_pos is not None
            and folded_pos[1] < rest_pos[1] - 0.03
            and folded_pos[2] < rest_pos[2] - 0.01,
            details=f"rest={rest_pos}, folded={folded_pos}",
        )

    left_side = object_model.get_part("left_side_antenna")
    left_joint = object_model.get_articulation("housing_to_left_side_antenna")
    left_rest = part_center(left_side)
    with ctx.pose({left_joint: 1.10}):
        left_folded = part_center(left_side)
    ctx.check(
        "left side antenna folds outward",
        left_rest is not None
        and left_folded is not None
        and left_folded[0] < left_rest[0] - 0.03
        and left_folded[2] < left_rest[2] - 0.01,
        details=f"rest={left_rest}, folded={left_folded}",
    )

    right_side = object_model.get_part("right_side_antenna")
    right_joint = object_model.get_articulation("housing_to_right_side_antenna")
    right_rest = part_center(right_side)
    with ctx.pose({right_joint: 1.10}):
        right_folded = part_center(right_side)
    ctx.check(
        "right side antenna folds outward",
        right_rest is not None
        and right_folded is not None
        and right_folded[0] > right_rest[0] + 0.03
        and right_folded[2] < right_rest[2] - 0.01,
        details=f"rest={right_rest}, folded={right_folded}",
    )

    knob_joint = object_model.get_articulation("housing_to_mode_knob")
    knob_rest = ctx.part_world_position(knob)
    with ctx.pose({knob_joint: 0.90}):
        knob_rotated = ctx.part_world_position(knob)
    ctx.check(
        "mode knob rotates in place",
        knob_rest is not None
        and knob_rotated is not None
        and abs(knob_rotated[0] - knob_rest[0]) < 1e-6
        and abs(knob_rotated[1] - knob_rest[1]) < 1e-6
        and abs(knob_rotated[2] - knob_rest[2]) < 1e-6,
        details=f"rest={knob_rest}, rotated={knob_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
