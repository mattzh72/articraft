from __future__ import annotations

from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


SHAFT_RADIUS = 0.022
PLATE_BORE_RADIUS = 0.034
COLLAR_BORE_RADIUS = 0.020


def _spoked_plate(
    *,
    outer_radius: float,
    rim_width: float,
    hub_radius: float,
    thickness: float,
    spokes: int,
    window_tangent: float,
    tab_angle: float,
) -> cq.Workplane:
    """One connected rotary plate with an open bore, rim, radial spokes, and an index tab."""
    plate = cq.Workplane("XY").circle(outer_radius).extrude(thickness).translate((0.0, 0.0, -thickness / 2.0))
    plate = plate.faces(">Z").workplane().circle(PLATE_BORE_RADIUS).cutThruAll()

    radial_gap = outer_radius - rim_width - hub_radius
    window_radial = radial_gap * 0.72
    window_center_radius = hub_radius + radial_gap * 0.50
    cutter_height = thickness * 3.0
    for index in range(spokes):
        angle = (360.0 / spokes) * (index + 0.5)
        cutter = (
            cq.Workplane("XY")
            .rect(window_radial, window_tangent)
            .extrude(cutter_height)
            .translate((window_center_radius, 0.0, -cutter_height / 2.0))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
        )
        plate = plate.cut(cutter)

    tab_length = min(0.030, outer_radius * 0.17)
    tab_width = min(0.040, outer_radius * 0.22)
    tab = (
        cq.Workplane("XY")
        .box(tab_length, tab_width, thickness * 1.12, centered=(True, True, True))
        .translate((outer_radius + tab_length / 2.0 - rim_width * 0.40, 0.0, 0.0))
        .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), tab_angle)
    )
    return plate.union(tab)


def _collar(*, outer_radius: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .extrude(height)
        .faces(">Z")
        .workplane()
        .circle(COLLAR_BORE_RADIUS)
        .cutThruAll()
        .translate((0.0, 0.0, -height / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_coaxial_rotary_assembly")

    painted_frame = model.material("painted_frame", rgba=(0.13, 0.15, 0.16, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.29, 0.30, 1.0))
    shaft_steel = model.material("polished_shaft", rgba=(0.72, 0.74, 0.76, 1.0))
    spacer_brass = model.material("brass_spacers", rgba=(0.76, 0.58, 0.26, 1.0))
    lower_blue = model.material("lower_blue", rgba=(0.10, 0.34, 0.72, 1.0))
    middle_orange = model.material("middle_orange", rgba=(0.88, 0.38, 0.10, 1.0))
    upper_green = model.material("upper_green", rgba=(0.12, 0.55, 0.30, 1.0))

    base = model.part("base")
    base.visual(Box((0.62, 0.42, 0.040)), origin=Origin(xyz=(0.0, 0.0, 0.020)), material=painted_frame, name="base_plate")
    base.visual(Box((0.56, 0.035, 0.050)), origin=Origin(xyz=(0.0, 0.185, 0.055)), material=painted_frame, name="front_rail")
    base.visual(Box((0.56, 0.035, 0.050)), origin=Origin(xyz=(0.0, -0.185, 0.055)), material=painted_frame, name="rear_rail")
    for x in (-0.285, 0.285):
        base.visual(
            Cylinder(radius=0.016, length=0.590),
            origin=Origin(xyz=(x, 0.0, 0.335)),
            material=painted_frame,
            name=f"side_post_{'neg' if x < 0 else 'pos'}",
        )
    base.visual(Box((0.62, 0.055, 0.045)), origin=Origin(xyz=(0.0, 0.0, 0.640)), material=painted_frame, name="top_bridge")
    base.visual(Cylinder(radius=SHAFT_RADIUS, length=0.600), origin=Origin(xyz=(0.0, 0.0, 0.340)), material=shaft_steel, name="fixed_shaft")
    base.visual(Cylinder(radius=0.058, length=0.045), origin=Origin(xyz=(0.0, 0.0, 0.063)), material=dark_steel, name="lower_bearing")
    base.visual(Cylinder(radius=0.055, length=0.045), origin=Origin(xyz=(0.0, 0.0, 0.618)), material=dark_steel, name="upper_bearing")
    for index, z in enumerate((0.100, 0.230, 0.390, 0.550)):
        base.visual(
            Cylinder(radius=0.029, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=spacer_brass,
            name=f"spacer_{index}",
        )

    member_specs = (
        ("outer_member", 0.205, 0.022, 0.056, 0.150, 6, 0.060, 0.0, lower_blue),
        ("middle_member", 0.165, 0.019, 0.052, 0.310, 5, 0.056, 24.0, middle_orange),
        ("inner_member", 0.125, 0.016, 0.048, 0.470, 4, 0.050, 45.0, upper_green),
    )
    for name, radius, rim_width, hub_radius, z, spokes, window_tangent, tab_angle, material in member_specs:
        member = model.part(name)
        plate = _spoked_plate(
            outer_radius=radius,
            rim_width=rim_width,
            hub_radius=hub_radius,
            thickness=0.018,
            spokes=spokes,
            window_tangent=window_tangent,
            tab_angle=tab_angle,
        )
        member.visual(
            mesh_from_cadquery(plate, f"{name}_spoked_plate", tolerance=0.001, angular_tolerance=0.08),
            material=material,
            name="spoked_plate",
        )
        member.visual(
            mesh_from_cadquery(_collar(outer_radius=hub_radius, height=0.052), f"{name}_collar", tolerance=0.001, angular_tolerance=0.08),
            material=dark_steel,
            name="split_collar",
        )
        member.visual(
            Cylinder(radius=0.006, length=0.026),
            origin=Origin(
                xyz=(
                    (radius - rim_width * 0.55) * cos(tab_angle * pi / 180.0),
                    (radius - rim_width * 0.55) * sin(tab_angle * pi / 180.0),
                    0.020,
                )
            ),
            material=shaft_steel,
            name="index_pin",
        )
        model.articulation(
            f"base_to_{name}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=member,
            origin=Origin(xyz=(0.0, 0.0, z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.4, velocity=5.0, lower=-pi, upper=pi),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    outer = object_model.get_part("outer_member")
    middle = object_model.get_part("middle_member")
    inner = object_model.get_part("inner_member")
    joints = [
        object_model.get_articulation("base_to_outer_member"),
        object_model.get_articulation("base_to_middle_member"),
        object_model.get_articulation("base_to_inner_member"),
    ]

    for joint in joints:
        ctx.check(
            f"{joint.name}_uses_shared_z_axis",
            tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0),
            details=f"axis={joint.axis!r}",
        )

    positions = [ctx.part_world_position(part) for part in (outer, middle, inner)]
    ctx.check("members_share_centerline", all(pos is not None and abs(pos[0]) < 1e-6 and abs(pos[1]) < 1e-6 for pos in positions), details=f"positions={positions!r}")
    ctx.expect_gap(middle, outer, axis="z", min_gap=0.085, name="lower_and_middle_collars_are_spaced")
    ctx.expect_gap(inner, middle, axis="z", min_gap=0.085, name="middle_and_upper_collars_are_spaced")
    for member in (outer, middle, inner):
        ctx.allow_overlap(
            base,
            member,
            elem_a="fixed_shaft",
            elem_b="split_collar",
            reason="The split collar is modeled as a slight interference bearing around the fixed shaft so the revolute member is visibly captured and supported.",
        )
        ctx.expect_overlap(
            base,
            member,
            axes="z",
            elem_a="fixed_shaft",
            elem_b="split_collar",
            min_overlap=0.045,
            name=f"{member.name}_collar_captures_shaft_length",
        )
    ctx.expect_overlap(outer, base, axes="xy", elem_a="split_collar", elem_b="fixed_shaft", min_overlap=0.010, name="outer_member_surrounds_fixed_shaft")
    ctx.expect_overlap(middle, base, axes="xy", elem_a="split_collar", elem_b="fixed_shaft", min_overlap=0.010, name="middle_member_surrounds_fixed_shaft")
    ctx.expect_overlap(inner, base, axes="xy", elem_a="split_collar", elem_b="fixed_shaft", min_overlap=0.010, name="inner_member_surrounds_fixed_shaft")

    rest_outer = ctx.part_element_world_aabb(outer, elem="index_pin")
    with ctx.pose({"base_to_outer_member": pi / 2.0, "base_to_middle_member": -pi / 3.0, "base_to_inner_member": pi / 4.0}):
        turned_outer = ctx.part_element_world_aabb(outer, elem="index_pin")
        turned_positions = [ctx.part_world_position(part) for part in (outer, middle, inner)]
    ctx.check(
        "independent_rotations_keep_shared_axis",
        all(pos is not None and abs(pos[0]) < 1e-6 and abs(pos[1]) < 1e-6 for pos in turned_positions),
        details=f"turned_positions={turned_positions!r}",
    )
    if rest_outer is not None and turned_outer is not None:
        rest_center = tuple((rest_outer[0][i] + rest_outer[1][i]) * 0.5 for i in range(3))
        turned_center = tuple((turned_outer[0][i] + turned_outer[1][i]) * 0.5 for i in range(3))
        ctx.check(
            "outer_index_pin_sweeps_around_axis",
            rest_center[0] > 0.15 and turned_center[1] > 0.15,
            details=f"rest_center={rest_center!r}, turned_center={turned_center!r}",
        )

    return ctx.report()


object_model = build_object_model()
