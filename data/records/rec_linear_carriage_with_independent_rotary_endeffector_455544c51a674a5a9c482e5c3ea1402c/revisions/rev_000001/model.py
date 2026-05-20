from __future__ import annotations

from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, *, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (radius * cos(2.0 * pi * i / segments), radius * sin(2.0 * pi * i / segments))
        for i in range(segments)
    ]


def _bearing_ring_mesh(name: str):
    ring = ExtrudeWithHolesGeometry(
        _circle_profile(0.025, segments=72),
        [_circle_profile(0.016, segments=56)],
        height=0.018,
        center=True,
    ).rotate_y(pi / 2.0)
    return mesh_from_geometry(ring, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_body_slide_rotary_nose")

    cast_iron = model.material("cast_iron", rgba=(0.19, 0.21, 0.22, 1.0))
    ground_steel = model.material("ground_steel", rgba=(0.63, 0.66, 0.68, 1.0))
    dark_oxide = model.material("dark_oxide", rgba=(0.05, 0.055, 0.06, 1.0))
    carriage_paint = model.material("carriage_paint", rgba=(0.08, 0.18, 0.28, 1.0))
    bronze = model.material("bronze_wear", rgba=(0.75, 0.48, 0.19, 1.0))
    nose_steel = model.material("nose_steel", rgba=(0.80, 0.80, 0.76, 1.0))
    marker_red = model.material("marker_red", rgba=(0.82, 0.08, 0.05, 1.0))

    guide = model.part("guide")
    guide.inertial = Inertial.from_geometry(Box((0.46, 0.20, 0.08)), mass=3.5)
    guide.visual(
        Box((0.46, 0.20, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0125)),
        material=cast_iron,
        name="base_plate",
    )
    for index, y, rail_name in ((0, 0.075, "rail_0"), (1, -0.075, "rail_1")):
        guide.visual(
            Box((0.44, 0.040, 0.055)),
            origin=Origin(xyz=(0.0, y, 0.0525)),
            material=ground_steel,
            name=rail_name,
        )
        for screw_index, x in enumerate((-0.165, -0.055, 0.055, 0.165)):
            guide.visual(
                Cylinder(radius=0.0065, length=0.006),
                origin=Origin(xyz=(x, y, 0.0825)),
                material=dark_oxide,
                name=f"bolt_{index}_{screw_index}",
            )
    for end_index, x in enumerate((-0.219, 0.219)):
        for rail_index, y in enumerate((0.075, -0.075)):
            guide.visual(
                Box((0.020, 0.052, 0.060)),
                origin=Origin(xyz=(x, y, 0.055)),
                material=cast_iron,
                name=f"end_stop_{end_index}_{rail_index}",
            )

    carriage = model.part("carriage")
    carriage.inertial = Inertial.from_geometry(
        Box((0.125, 0.108, 0.058)),
        mass=0.9,
        origin=Origin(xyz=(0.006, 0.0, 0.006)),
    )
    carriage.visual(
        Box((0.105, 0.096, 0.040)),
        material=carriage_paint,
        name="saddle_block",
    )
    carriage.visual(
        Box((0.115, 0.108, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=carriage_paint,
        name="top_cap",
    )
    carriage.visual(
        Box((0.090, 0.007, 0.026)),
        origin=Origin(xyz=(0.0, 0.0515, -0.004)),
        material=bronze,
        name="wear_pad_0",
    )
    carriage.visual(
        Box((0.090, 0.007, 0.026)),
        origin=Origin(xyz=(0.0, -0.0515, -0.004)),
        material=bronze,
        name="wear_pad_1",
    )
    carriage.visual(
        Box((0.012, 0.058, 0.044)),
        origin=Origin(xyz=(0.0525, 0.0, 0.004)),
        material=carriage_paint,
        name="front_boss",
    )
    carriage.visual(
        _bearing_ring_mesh("carriage_bearing_ring"),
        origin=Origin(xyz=(0.064, 0.0, 0.005)),
        material=ground_steel,
        name="bearing_ring",
    )

    nose = model.part("nose")
    nose.inertial = Inertial.from_geometry(
        Cylinder(radius=0.027, length=0.090),
        mass=0.28,
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )
    nose.visual(
        Cylinder(radius=0.014, length=0.035),
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=nose_steel,
        name="rear_stub",
    )
    nose.visual(
        Cylinder(radius=0.017, length=0.058),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=nose_steel,
        name="cartridge_body",
    )
    nose.visual(
        Cylinder(radius=0.027, length=0.010),
        origin=Origin(xyz=(0.074, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=nose_steel,
        name="flange",
    )
    nose.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.082, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_oxide,
        name="front_cap",
    )
    nose.visual(
        Box((0.008, 0.010, 0.010)),
        origin=Origin(xyz=(0.074, 0.0, 0.029)),
        material=marker_red,
        name="index_tab",
    )

    model.articulation(
        "carriage_slide",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=carriage,
        origin=Origin(xyz=(-0.090, 0.0, 0.064)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.35, lower=0.0, upper=0.180),
    )
    model.articulation(
        "nose_spin",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=nose,
        origin=Origin(xyz=(0.064, 0.0, 0.005)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=6.0, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide = object_model.get_part("guide")
    carriage = object_model.get_part("carriage")
    nose = object_model.get_part("nose")
    slide = object_model.get_articulation("carriage_slide")
    spin = object_model.get_articulation("nose_spin")

    ctx.allow_overlap(
        carriage,
        nose,
        elem_a="bearing_ring",
        elem_b="rear_stub",
        reason="The rotary rear stub is intentionally captured inside the carriage bearing ring.",
    )

    ctx.check("guide_carriage_nose_present", all((guide, carriage, nose, slide, spin)))
    ctx.check(
        "primary_joint_types",
        slide.articulation_type == ArticulationType.PRISMATIC and spin.articulation_type == ArticulationType.REVOLUTE,
        details=f"slide={slide.articulation_type}, spin={spin.articulation_type}",
    )

    with ctx.pose({slide: 0.0, spin: 0.0}):
        ctx.expect_contact(
            carriage,
            guide,
            elem_a="wear_pad_0",
            elem_b="rail_0",
            contact_tol=0.0005,
            name="upper wear pad is carried by the split rail",
        )
        ctx.expect_contact(
            carriage,
            guide,
            elem_a="wear_pad_1",
            elem_b="rail_1",
            contact_tol=0.0005,
            name="lower wear pad is carried by the split rail",
        )
        ctx.expect_within(
            nose,
            carriage,
            axes="yz",
            inner_elem="rear_stub",
            outer_elem="bearing_ring",
            margin=0.0025,
            name="rotary stub is centered in the carriage bearing",
        )
        ctx.expect_overlap(
            nose,
            carriage,
            axes="x",
            elem_a="rear_stub",
            elem_b="bearing_ring",
            min_overlap=0.015,
            name="rotary stub remains inserted through the bearing",
        )
        rest_pos = ctx.part_world_position(carriage)
        tab_rest_aabb = ctx.part_element_world_aabb(nose, elem="index_tab")

    with ctx.pose({slide: 0.180, spin: 0.0}):
        ctx.expect_contact(
            carriage,
            guide,
            elem_a="wear_pad_0",
            elem_b="rail_0",
            contact_tol=0.0005,
            name="extended carriage remains supported on rail 0",
        )
        ctx.expect_contact(
            carriage,
            guide,
            elem_a="wear_pad_1",
            elem_b="rail_1",
            contact_tol=0.0005,
            name="extended carriage remains supported on rail 1",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates along guide",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.16,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({slide: 0.0, spin: pi / 2.0}):
        tab_rotated_aabb = ctx.part_element_world_aabb(nose, elem="index_tab")

    if tab_rest_aabb is not None and tab_rotated_aabb is not None:
        rest_center_y = (tab_rest_aabb[0][1] + tab_rest_aabb[1][1]) * 0.5
        rotated_center_y = (tab_rotated_aabb[0][1] + tab_rotated_aabb[1][1]) * 0.5
        rest_center_z = (tab_rest_aabb[0][2] + tab_rest_aabb[1][2]) * 0.5
        rotated_center_z = (tab_rotated_aabb[0][2] + tab_rotated_aabb[1][2]) * 0.5
        ctx.check(
            "nose rotation moves the flange index tab",
            abs(rotated_center_y - rest_center_y) > 0.020 and abs(rotated_center_z - rest_center_z) > 0.020,
            details=(
                f"rest_yz=({rest_center_y:.4f}, {rest_center_z:.4f}), "
                f"rotated_yz=({rotated_center_y:.4f}, {rotated_center_z:.4f})"
            ),
        )
    else:
        ctx.fail("nose rotation moves the flange index tab", "index_tab AABB was unavailable")

    return ctx.report()


object_model = build_object_model()
