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
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="maelzel_pyramid_metronome")

    walnut = model.material("walnut", rgba=(0.42, 0.24, 0.12, 1.0))
    ebonized = model.material("ebonized_trim", rgba=(0.14, 0.08, 0.05, 1.0))
    brass = model.material("brass", rgba=(0.77, 0.65, 0.28, 1.0))
    steel = model.material("steel", rgba=(0.70, 0.72, 0.75, 1.0))

    housing = model.part("housing")
    shell_mesh = mesh_from_geometry(_build_housing_shell_mesh(), "metronome_shell")
    housing.visual(shell_mesh, material=walnut, name="shell")
    housing.visual(
        Box((0.134, 0.124, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=ebonized,
        name="base_plinth",
    )
    housing.visual(
        Box((0.094, 0.088, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        material=walnut,
        name="inner_floor",
    )
    housing.visual(
        Box((0.020, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.019, 0.170)),
        material=ebonized,
        name="shaft_housing",
    )
    housing.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(0.0, -0.046, 0.043), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="key_bushing",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.134, 0.124, 0.245)),
        mass=0.78,
        origin=Origin(xyz=(0.0, 0.0, 0.1225)),
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.0025, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_axle",
    )
    pendulum.visual(
        Box((0.0045, 0.0018, 0.248)),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material=steel,
        name="rod",
    )
    pendulum.visual(
        Box((0.006, 0.032, 0.005)),
        origin=Origin(xyz=(0.0, 0.0169, -0.118)),
        material=brass,
        name="counterweight_bridge",
    )
    pendulum.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.0, 0.040, -0.118), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="counterweight",
    )
    pendulum.inertial = Inertial.from_geometry(
        Box((0.040, 0.038, 0.248)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.012, -0.080)),
    )

    slider = model.part("slider_weight")
    slider.visual(
        Box((0.006, 0.0162, 0.008)),
        origin=Origin(xyz=(0.0, 0.009, 0.008)),
        material=brass,
        name="slider_bridge",
    )
    slider_cone = ConeGeometry(radius=0.014, height=0.028)
    slider_cone.rotate_x(math.pi / 2.0)
    slider_cone.translate(0.0, 0.024, 0.012)
    slider.visual(
        mesh_from_geometry(slider_cone, "slider_cone"),
        material=brass,
        name="slider_cone",
    )
    slider.inertial = Inertial.from_geometry(
        Box((0.028, 0.038, 0.028)),
        mass=0.025,
        origin=Origin(xyz=(0.0, 0.020, 0.012)),
    )

    key = model.part("winding_key")
    key.visual(
        Cylinder(radius=0.0036, length=0.024),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="key_stem",
    )
    key.visual(
        Box((0.026, 0.006, 0.006)),
        origin=Origin(xyz=(0.0, -0.024, 0.0)),
        material=brass,
        name="key_handle",
    )
    key.visual(
        Box((0.006, 0.012, 0.006)),
        origin=Origin(xyz=(-0.013, -0.024, 0.0)),
        material=brass,
        name="left_wing",
    )
    key.visual(
        Box((0.006, 0.012, 0.006)),
        origin=Origin(xyz=(0.013, -0.024, 0.0)),
        material=brass,
        name="right_wing",
    )
    key.inertial = Inertial.from_geometry(
        Box((0.032, 0.036, 0.008)),
        mass=0.015,
        origin=Origin(xyz=(0.0, -0.020, 0.0)),
    )

    model.articulation(
        "housing_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=pendulum,
        origin=Origin(xyz=(0.0, 0.022, 0.170)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=2.5,
            lower=-0.42,
            upper=0.42,
        ),
    )
    model.articulation(
        "pendulum_to_slider",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=slider,
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.3,
            velocity=0.12,
            lower=0.0,
            upper=0.074,
        ),
    )
    model.articulation(
        "housing_to_key",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=key,
        origin=Origin(xyz=(0.0, -0.049, 0.043)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.1, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    pendulum = object_model.get_part("pendulum")
    slider = object_model.get_part("slider_weight")
    key = object_model.get_part("winding_key")

    pendulum_joint = object_model.get_articulation("housing_to_pendulum")
    slider_joint = object_model.get_articulation("pendulum_to_slider")
    key_joint = object_model.get_articulation("housing_to_key")

    ctx.expect_origin_distance(
        pendulum,
        housing,
        axes="x",
        max_dist=0.001,
        name="pendulum is centered on the housing slot",
    )
    ctx.expect_origin_gap(
        slider,
        pendulum,
        axis="z",
        min_gap=0.01,
        name="slider weight starts above the pendulum pivot",
    )
    ctx.expect_origin_distance(
        slider,
        pendulum,
        axes="xy",
        max_dist=0.001,
        name="slider stays aligned to the pendulum rod axis at rest",
    )

    slider_rest = ctx.part_world_position(slider)
    with ctx.pose({slider_joint: slider_joint.motion_limits.upper}):
        ctx.expect_origin_distance(
            slider,
            pendulum,
            axes="xy",
            max_dist=0.001,
            name="slider stays aligned to the pendulum rod axis when raised",
        )
        slider_high = ctx.part_world_position(slider)
    ctx.check(
        "slider weight travels upward along the rod",
        slider_rest is not None
        and slider_high is not None
        and slider_high[2] > slider_rest[2] + 0.06,
        details=f"rest={slider_rest}, raised={slider_high}",
    )

    pendulum_rest = ctx.part_element_world_aabb(pendulum, elem="counterweight")
    with ctx.pose({pendulum_joint: pendulum_joint.motion_limits.upper}):
        pendulum_swung = ctx.part_element_world_aabb(pendulum, elem="counterweight")
    ctx.check(
        "pendulum swings laterally at positive angle",
        pendulum_rest is not None
        and pendulum_swung is not None
        and (pendulum_swung[0][0] + pendulum_swung[1][0]) * 0.5
        > (pendulum_rest[0][0] + pendulum_rest[1][0]) * 0.5 + 0.035,
        details=f"rest={pendulum_rest}, swung={pendulum_swung}",
    )

    ctx.check(
        "winding key uses continuous rotation",
        key_joint.articulation_type == ArticulationType.CONTINUOUS
        and key_joint.motion_limits is not None
        and key_joint.motion_limits.lower is None
        and key_joint.motion_limits.upper is None,
        details=f"joint_type={key_joint.articulation_type}, limits={key_joint.motion_limits}",
    )
    ctx.check(
        "winding key protrudes from the rear face",
        ctx.part_world_position(key) is not None
        and ctx.part_world_position(housing) is not None
        and ctx.part_world_position(key)[1] < ctx.part_world_position(housing)[1] - 0.02,
        details=f"key={ctx.part_world_position(key)}, housing={ctx.part_world_position(housing)}",
    )

    return ctx.report()


def _add_quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _add_panel_prism(
    geom: MeshGeometry,
    outer_loop: list[tuple[float, float, float]],
    inner_loop: list[tuple[float, float, float]],
) -> None:
    outer_ids = [geom.add_vertex(*point) for point in outer_loop]
    inner_ids = [geom.add_vertex(*point) for point in inner_loop]
    count = len(outer_ids)

    for index in range(1, count - 1):
        geom.add_face(outer_ids[0], outer_ids[index], outer_ids[index + 1])
        geom.add_face(inner_ids[0], inner_ids[index + 1], inner_ids[index])

    for index in range(count):
        next_index = (index + 1) % count
        _add_quad(
            geom,
            outer_ids[index],
            outer_ids[next_index],
            inner_ids[next_index],
            inner_ids[index],
        )


def _section_dims(
    z: float,
    *,
    z_base: float,
    z_top: float,
    width_base: float,
    width_top: float,
    depth_base: float,
    depth_top: float,
) -> tuple[float, float]:
    fraction = (z - z_base) / (z_top - z_base)
    width = width_base + (width_top - width_base) * fraction
    depth = depth_base + (depth_top - depth_base) * fraction
    return width, depth


def _build_housing_shell_mesh() -> MeshGeometry:
    geom = MeshGeometry()

    z_base = 0.018
    z_slot = 0.040
    z_top = 0.235
    shell_thickness = 0.006
    width_base = 0.118
    width_top = 0.028
    depth_base = 0.102
    depth_top = 0.030

    width_slot, depth_slot = _section_dims(
        z_slot,
        z_base=z_base,
        z_top=z_top,
        width_base=width_base,
        width_top=width_top,
        depth_base=depth_base,
        depth_top=depth_top,
    )

    inner_width_base = width_base - 2.0 * shell_thickness
    inner_width_slot = width_slot - 2.0 * shell_thickness
    inner_width_top = width_top - 2.0 * shell_thickness
    inner_depth_base = depth_base - 2.0 * shell_thickness
    inner_depth_slot = depth_slot - 2.0 * shell_thickness
    inner_depth_top = depth_top - 2.0 * shell_thickness

    slot_width_outer_slot = 0.028
    slot_width_outer_top = 0.012
    slot_width_inner_slot = 0.036
    slot_width_inner_top = 0.020

    half_w_base = width_base * 0.5
    half_w_slot = width_slot * 0.5
    half_w_top = width_top * 0.5
    half_d_base = depth_base * 0.5
    half_d_slot = depth_slot * 0.5
    half_d_top = depth_top * 0.5

    half_wi_base = inner_width_base * 0.5
    half_wi_slot = inner_width_slot * 0.5
    half_wi_top = inner_width_top * 0.5
    half_di_base = inner_depth_base * 0.5
    half_di_slot = inner_depth_slot * 0.5
    half_di_top = inner_depth_top * 0.5

    back_outer = [
        (-half_w_base, -half_d_base, z_base),
        (half_w_base, -half_d_base, z_base),
        (half_w_top, -half_d_top, z_top),
        (-half_w_top, -half_d_top, z_top),
    ]
    back_inner = [
        (-half_wi_base, -half_di_base, z_base),
        (half_wi_base, -half_di_base, z_base),
        (half_wi_top, -half_di_top, z_top),
        (-half_wi_top, -half_di_top, z_top),
    ]
    _add_panel_prism(geom, back_outer, back_inner)

    left_outer = [
        (-half_w_base, -half_d_base, z_base),
        (-half_w_base, half_d_base, z_base),
        (-half_w_top, half_d_top, z_top),
        (-half_w_top, -half_d_top, z_top),
    ]
    left_inner = [
        (-half_wi_base, -half_di_base, z_base),
        (-half_wi_base, half_di_base, z_base),
        (-half_wi_top, half_di_top, z_top),
        (-half_wi_top, -half_di_top, z_top),
    ]
    _add_panel_prism(geom, left_outer, left_inner)

    right_outer = [
        (half_w_base, half_d_base, z_base),
        (half_w_base, -half_d_base, z_base),
        (half_w_top, -half_d_top, z_top),
        (half_w_top, half_d_top, z_top),
    ]
    right_inner = [
        (half_wi_base, half_di_base, z_base),
        (half_wi_base, -half_di_base, z_base),
        (half_wi_top, -half_di_top, z_top),
        (half_wi_top, half_di_top, z_top),
    ]
    _add_panel_prism(geom, right_outer, right_inner)

    lower_apron_outer = [
        (-half_w_base, half_d_base, z_base),
        (half_w_base, half_d_base, z_base),
        (half_w_slot, half_d_slot, z_slot),
        (-half_w_slot, half_d_slot, z_slot),
    ]
    lower_apron_inner = [
        (-half_wi_base, half_di_base, z_base),
        (half_wi_base, half_di_base, z_base),
        (half_wi_slot, half_di_slot, z_slot),
        (-half_wi_slot, half_di_slot, z_slot),
    ]
    _add_panel_prism(geom, lower_apron_outer, lower_apron_inner)

    left_rail_outer = [
        (-half_w_slot, half_d_slot, z_slot),
        (-slot_width_outer_slot * 0.5, half_d_slot, z_slot),
        (-slot_width_outer_top * 0.5, half_d_top, z_top),
        (-half_w_top, half_d_top, z_top),
    ]
    left_rail_inner = [
        (-half_wi_slot, half_di_slot, z_slot),
        (-slot_width_inner_slot * 0.5, half_di_slot, z_slot),
        (-slot_width_inner_top * 0.5, half_di_top, z_top),
        (-half_wi_top, half_di_top, z_top),
    ]
    _add_panel_prism(geom, left_rail_outer, left_rail_inner)

    right_rail_outer = [
        (slot_width_outer_slot * 0.5, half_d_slot, z_slot),
        (half_w_slot, half_d_slot, z_slot),
        (half_w_top, half_d_top, z_top),
        (slot_width_outer_top * 0.5, half_d_top, z_top),
    ]
    right_rail_inner = [
        (slot_width_inner_slot * 0.5, half_di_slot, z_slot),
        (half_wi_slot, half_di_slot, z_slot),
        (half_wi_top, half_di_top, z_top),
        (slot_width_inner_top * 0.5, half_di_top, z_top),
    ]
    _add_panel_prism(geom, right_rail_outer, right_rail_inner)

    return geom


# >>> USER_CODE_END

object_model = build_object_model()
