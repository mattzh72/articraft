from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


BURNER_LAYOUT = (
    ("rear_burner_0", -0.205, 0.115, 0.054),
    ("rear_burner_1", 0.205, 0.115, 0.068),
    ("front_burner_0", -0.205, -0.125, 0.074),
    ("front_burner_1", 0.205, -0.125, 0.056),
)

KNOB_LAYOUT = (
    ("knob_0_0", -0.030, -0.344, 0.120),
    ("knob_0_1", 0.030, -0.344, 0.120),
    ("knob_1_0", -0.030, -0.344, 0.180),
    ("knob_1_1", 0.030, -0.344, 0.180),
)


def _rounded_slab(width: float, depth: float, height: float, radius: float):
    return ExtrudeGeometry.from_z0(
        rounded_rect_profile(width, depth, radius, corner_segments=8),
        height,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flush_gas_cooktop")

    stone = model.material("warm_stone", rgba=(0.70, 0.64, 0.54, 1.0))
    dark_glass = model.material("black_ceramic_glass", rgba=(0.025, 0.027, 0.030, 0.96))
    black_cast = model.material("black_cast_iron", rgba=(0.015, 0.015, 0.014, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.64, 0.65, 0.62, 1.0))
    shadow = model.material("cutout_shadow", rgba=(0.006, 0.006, 0.005, 1.0))
    white_mark = model.material("white_control_mark", rgba=(0.92, 0.90, 0.82, 1.0))

    countertop = model.part("countertop")
    countertop.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(1.000, 0.760, 0.018, corner_segments=8),
                [rounded_rect_profile(0.745, 0.545, 0.020, corner_segments=8)],
                height=0.045,
                center=True,
            ),
            "countertop_cutout_frame",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=stone,
        name="countertop_cutout",
    )
    countertop.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(0.760, 0.560, 0.020, corner_segments=8),
                [rounded_rect_profile(0.705, 0.505, 0.016, corner_segments=8)],
                height=0.012,
                center=True,
            ),
            "recess_support_lip",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=shadow,
        name="support_lip",
    )
    countertop.visual(
        mesh_from_geometry(
            _rounded_slab(0.725, 0.525, 0.014, 0.024),
            "black_glass_insert",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.043)),
        material=dark_glass,
        name="cooktop_glass",
    )
    countertop.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                rounded_rect_profile(0.735, 0.535, 0.024, corner_segments=8),
                [rounded_rect_profile(0.715, 0.515, 0.020, corner_segments=8)],
                height=0.003,
                center=True,
            ),
            "thin_black_gasket",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=shadow,
        name="perimeter_gasket",
    )

    countertop.visual(
        Box((0.220, 0.075, 0.030)),
        origin=Origin(xyz=(0.0, -0.288, 0.072)),
        material=stainless,
        name="control_plinth",
    )
    countertop.visual(
        Box((0.190, 0.018, 0.165)),
        origin=Origin(xyz=(0.0, -0.329, 0.148)),
        material=stainless,
        name="control_face",
    )

    for knob_name, x, y, z in KNOB_LAYOUT:
        countertop.visual(
            Cylinder(radius=0.022, length=0.006),
            origin=Origin(xyz=(x, y + 0.003, z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=black_cast,
            name=f"{knob_name}_bezel",
        )

    for burner_name, x, y, radius in BURNER_LAYOUT:
        cap_radius = radius * 0.42
        countertop.visual(
            Cylinder(radius=radius * 1.12, length=0.004),
            origin=Origin(xyz=(x, y, 0.060)),
            material=stainless,
            name=f"{burner_name}_tray",
        )
        countertop.visual(
            mesh_from_geometry(
                TorusGeometry(radius=radius * 0.78, tube=0.0045, radial_segments=20, tubular_segments=56),
                f"{burner_name}_gas_ring",
            ),
            origin=Origin(xyz=(x, y, 0.066)),
            material=black_cast,
            name=f"{burner_name}_ring",
        )
        countertop.visual(
            Cylinder(radius=cap_radius, length=0.010),
            origin=Origin(xyz=(x, y, 0.071)),
            material=black_cast,
            name=f"{burner_name}_cap",
        )
        grate_span = radius * 2.85
        countertop.visual(
            Box((grate_span, 0.018, 0.015)),
            origin=Origin(xyz=(x, y, 0.075)),
            material=black_cast,
            name=f"{burner_name}_grate_x",
        )
        countertop.visual(
            Box((0.018, grate_span, 0.015)),
            origin=Origin(xyz=(x, y, 0.075)),
            material=black_cast,
            name=f"{burner_name}_grate_y",
        )
        foot_offset = radius * 1.22
        for foot_index, (dx, dy) in enumerate(
            ((foot_offset, 0.0), (-foot_offset, 0.0), (0.0, foot_offset), (0.0, -foot_offset))
        ):
            countertop.visual(
                Box((0.020, 0.020, 0.016)),
                origin=Origin(xyz=(x + dx, y + dy, 0.066)),
                material=black_cast,
                name=f"{burner_name}_grate_foot_{foot_index}",
            )

    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.036,
            0.022,
            body_style="skirted",
            top_diameter=0.030,
            skirt=KnobSkirt(0.042, 0.004, flare=0.05, chamfer=0.0008),
            grip=KnobGrip(style="fluted", count=16, depth=0.0010),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0006),
            bore=KnobBore(style="round", diameter=0.006),
            center=False,
        ),
        "front_rotary_knob",
    )

    for knob_name, x, y, z in KNOB_LAYOUT:
        knob = model.part(knob_name)
        knob.visual(
            knob_mesh,
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
            material=black_cast,
            name="knob_body",
        )
        knob.visual(
            Box((0.004, 0.0014, 0.020)),
            origin=Origin(xyz=(0.0, -0.0222, 0.006)),
            material=white_mark,
            name="pointer",
        )
        knob.inertial = Inertial.from_geometry(
            Cylinder(radius=0.021, length=0.024),
            mass=0.055,
            origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        )
        model.articulation(
            f"{knob_name}_spin",
            ArticulationType.CONTINUOUS,
            parent=countertop,
            child=knob,
            origin=Origin(xyz=(x, y, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.35, velocity=9.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    countertop = object_model.get_part("countertop")
    knob_names = [name for name, _, _, _ in KNOB_LAYOUT]
    joints = [object_model.get_articulation(f"{name}_spin") for name in knob_names]

    ctx.check(
        "only_four_knob_articulations",
        len(object_model.articulations) == 4
        and all(joint.child in knob_names for joint in object_model.articulations),
        details=f"articulations={[joint.name for joint in object_model.articulations]!r}",
    )
    for joint in joints:
        axis = tuple(float(value) for value in joint.axis)
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name}_continuous_front_back_axis",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and abs(axis[0]) < 1e-6
            and abs(abs(axis[1]) - 1.0) < 1e-6
            and abs(axis[2]) < 1e-6
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, axis={axis}, limits={limits}",
        )

    positions = [ctx.part_world_position(object_model.get_part(name)) for name in knob_names]
    if all(position is not None for position in positions):
        xs = [position[0] for position in positions if position is not None]
        ys = [position[1] for position in positions if position is not None]
        zs = [position[2] for position in positions if position is not None]
        ctx.check(
            "knobs_form_front_center_square",
            abs(sum(xs) / len(xs)) < 0.002
            and max(ys) < -0.32
            and 0.055 <= (max(xs) - min(xs)) <= 0.070
            and 0.055 <= (max(zs) - min(zs)) <= 0.070,
            details=f"positions={positions!r}",
        )
    else:
        ctx.fail("knobs_form_front_center_square", f"positions={positions!r}")

    for knob_name in knob_names:
        knob = object_model.get_part(knob_name)
        ctx.expect_gap(
            countertop,
            knob,
            axis="y",
            max_gap=0.002,
            max_penetration=0.0005,
            positive_elem=f"{knob_name}_bezel",
            negative_elem="knob_body",
            name=f"{knob_name}_seats_against_front_bezel",
        )
        ctx.expect_overlap(
            countertop,
            knob,
            axes="xz",
            min_overlap=0.018,
            elem_a=f"{knob_name}_bezel",
            elem_b="knob_body",
            name=f"{knob_name}_aligned_with_bezel",
        )

    first_knob = object_model.get_part("knob_0_0")
    first_joint = object_model.get_articulation("knob_0_0_spin")
    rest_aabb = ctx.part_element_world_aabb(first_knob, elem="pointer")
    with ctx.pose({first_joint: pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(first_knob, elem="pointer")
    if rest_aabb is not None and turned_aabb is not None:
        rest_center = tuple((rest_aabb[0][i] + rest_aabb[1][i]) / 2.0 for i in range(3))
        turned_center = tuple((turned_aabb[0][i] + turned_aabb[1][i]) / 2.0 for i in range(3))
        ctx.check(
            "knob_pointer_rotates_about_front_back_axis",
            turned_center[0] > rest_center[0] + 0.004
            and abs(turned_center[1] - rest_center[1]) < 0.002
            and turned_center[2] < rest_center[2] - 0.002,
            details=f"rest={rest_center!r}, turned={turned_center!r}",
        )
    else:
        ctx.fail("knob_pointer_rotates_about_front_back_axis", f"rest={rest_aabb}, turned={turned_aabb}")

    return ctx.report()


object_model = build_object_model()
