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
)


def axis_origin(axis: str, xyz: tuple[float, float, float]) -> Origin:
    if axis == "x":
        return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))
    if axis == "y":
        return Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0))
    return Origin(xyz=xyz)


def add_ribbed_cylinder(
    part,
    *,
    axis: str,
    xyz: tuple[float, float, float],
    radius: float,
    length: float,
    rib_radius: float,
    rib_length: float,
    rib_pitch: float,
    rib_count: int,
    material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=axis_origin(axis, xyz),
        material=material,
        name=name,
    )
    axis_index = {"x": 0, "y": 1, "z": 2}[axis]
    first_offset = -0.5 * rib_pitch * (rib_count - 1)
    for index in range(rib_count):
        coords = [xyz[0], xyz[1], xyz[2]]
        coords[axis_index] += first_offset + index * rib_pitch
        part.visual(
            Cylinder(radius=rib_radius, length=rib_length),
            origin=axis_origin(axis, tuple(coords)),
            material=material,
            name=f"{name}_rib_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="porro_field_binoculars")

    armor = model.material("armor", rgba=(0.16, 0.18, 0.16, 1.0))
    trim = model.material("trim", rgba=(0.10, 0.10, 0.11, 1.0))
    knurl = model.material("knurl", rgba=(0.13, 0.13, 0.14, 1.0))
    metal = model.material("metal", rgba=(0.25, 0.26, 0.28, 1.0))
    glass = model.material("glass", rgba=(0.34, 0.44, 0.48, 0.35))

    bridge = model.part("bridge")
    bridge.visual(
        Cylinder(radius=0.006, length=0.084),
        origin=Origin(xyz=(0.0, 0.006, 0.044)),
        material=metal,
        name="hinge_pin",
    )
    for index, z_pos in enumerate((0.058, 0.062, 0.066, 0.070, 0.074, 0.078)):
        bridge.visual(
            Cylinder(radius=0.0082, length=0.0026),
            origin=Origin(xyz=(0.0, 0.006, z_pos)),
            material=knurl,
            name=f"hinge_knurl_{index}",
        )
    bridge.visual(
        Cylinder(radius=0.0085, length=0.008),
        origin=Origin(xyz=(0.0, 0.006, 0.082)),
        material=trim,
        name="hinge_cap",
    )
    bridge.visual(
        Cylinder(radius=0.0085, length=0.010),
        origin=Origin(xyz=(0.0, 0.006, 0.010)),
        material=trim,
        name="hinge_foot",
    )
    bridge.visual(
        Cylinder(radius=0.0048, length=0.010),
        origin=axis_origin("x", (-0.014, -0.030, 0.092)),
        material=metal,
        name="left_focus_shaft",
    )
    bridge.visual(
        Cylinder(radius=0.0048, length=0.010),
        origin=axis_origin("x", (0.014, -0.030, 0.092)),
        material=metal,
        name="right_focus_shaft",
    )
    bridge.visual(
        Box((0.006, 0.024, 0.010)),
        origin=Origin(xyz=(-0.014, -0.018, 0.084)),
        material=trim,
        name="left_focus_arm",
    )
    bridge.visual(
        Box((0.006, 0.024, 0.010)),
        origin=Origin(xyz=(0.014, -0.018, 0.084)),
        material=trim,
        name="right_focus_arm",
    )
    bridge.visual(
        Box((0.030, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, -0.008, 0.082)),
        material=trim,
        name="focus_crossbar",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((0.060, 0.074, 0.092)),
        mass=0.34,
        origin=Origin(xyz=(0.0, -0.006, 0.046)),
    )

    def build_body(name: str, side: float):
        body = model.part(name)
        hinge_z = 0.007 * side
        body.visual(
            Cylinder(radius=0.0088, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, hinge_z)),
            material=trim,
            name="hinge_barrel",
        )
        body.visual(
            Box((0.020, 0.020, 0.018)),
            origin=Origin(xyz=(0.016 * side, 0.003, hinge_z)),
            material=trim,
            name="hinge_arm",
        )
        body.visual(
            Box((0.042, 0.056, 0.020)),
            origin=Origin(xyz=(0.040 * side, 0.008, -0.004)),
            material=armor,
            name="body_bridge",
        )
        body.visual(
            Box((0.044, 0.042, 0.032)),
            origin=Origin(xyz=(0.032 * side, -0.010, 0.004)),
            material=armor,
            name="rear_prism_housing",
        )
        body.visual(
            Box((0.052, 0.050, 0.036)),
            origin=Origin(xyz=(0.062 * side, 0.030, -0.004)),
            material=armor,
            name="front_prism_housing",
        )
        body.visual(
            Box((0.056, 0.062, 0.010)),
            origin=Origin(xyz=(0.046 * side, 0.012, 0.019)),
            material=trim,
            name="prism_cap",
        )
        body.visual(
            Box((0.024, 0.024, 0.024)),
            origin=Origin(xyz=(0.081 * side, 0.048, -0.011)),
            material=armor,
            name="objective_shoulder",
        )
        body.visual(
            Cylinder(radius=0.030, length=0.016),
            origin=axis_origin("y", (0.084 * side, 0.056, -0.012)),
            material=trim,
            name="objective_rear_ring",
        )
        body.visual(
            Cylinder(radius=0.028, length=0.058),
            origin=axis_origin("y", (0.084 * side, 0.093, -0.012)),
            material=armor,
            name="objective_barrel",
        )
        body.visual(
            Cylinder(radius=0.031, length=0.010),
            origin=axis_origin("y", (0.084 * side, 0.127, -0.012)),
            material=trim,
            name="objective_front_ring",
        )
        body.visual(
            Cylinder(radius=0.023, length=0.001),
            origin=axis_origin("y", (0.084 * side, 0.1315, -0.012)),
            material=glass,
            name="objective_lens",
        )
        body.visual(
            Box((0.028, 0.022, 0.016)),
            origin=Origin(xyz=(0.027 * side, -0.028, 0.010)),
            material=armor,
            name="eyepiece_bridge",
        )
        body.visual(
            Cylinder(radius=0.016, length=0.022),
            origin=axis_origin("y", (0.026 * side, -0.050, 0.014)),
            material=trim,
            name="eyepiece_tube",
        )
        body.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=axis_origin("y", (0.026 * side, -0.064, 0.014)),
            material=trim,
            name="diopter_seat",
        )
        body.inertial = Inertial.from_geometry(
            Box((0.130, 0.190, 0.070)),
            mass=0.42,
            origin=Origin(xyz=(0.046 * side, 0.020, 0.001)),
        )
        return body

    left_body = build_body("left_body", 1.0)
    right_body = build_body("right_body", -1.0)

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=axis_origin("x", (0.0, 0.0, 0.0)),
        material=metal,
        name="focus_core",
    )
    add_ribbed_cylinder(
        focus_knob,
        axis="x",
        xyz=(0.0, 0.0, 0.0),
        radius=0.0145,
        length=0.016,
        rib_radius=0.0158,
        rib_length=0.0016,
        rib_pitch=0.0026,
        rib_count=5,
        material=knurl,
        name="focus_wheel",
    )
    focus_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.015, length=0.018),
        mass=0.045,
        origin=axis_origin("x", (0.0, 0.0, 0.0)),
    )

    def build_diopter(name: str):
        ring = model.part(name)
        ring.visual(
            Cylinder(radius=0.018, length=0.010),
            origin=axis_origin("y", (0.0, 0.0, 0.0)),
            material=trim,
            name="ring_core",
        )
        add_ribbed_cylinder(
            ring,
            axis="y",
            xyz=(0.0, 0.0, 0.0),
            radius=0.0205,
            length=0.010,
            rib_radius=0.0215,
            rib_length=0.0012,
            rib_pitch=0.0022,
            rib_count=4,
            material=knurl,
            name="diopter_ring",
        )
        ring.visual(
            Cylinder(radius=0.020, length=0.014),
            origin=axis_origin("y", (0.0, -0.012, 0.0)),
            material=trim,
            name="eyecup",
        )
        ring.visual(
            Cylinder(radius=0.010, length=0.001),
            origin=axis_origin("y", (0.0, -0.0185, 0.0)),
            material=glass,
            name="eyepiece_lens",
        )
        ring.inertial = Inertial.from_geometry(
            Cylinder(radius=0.020, length=0.010),
            mass=0.015,
            origin=axis_origin("y", (0.0, 0.0, 0.0)),
        )
        return ring

    left_diopter = build_diopter("left_diopter")
    right_diopter = build_diopter("right_diopter")

    model.articulation(
        "left_hinge",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=left_body,
        origin=Origin(xyz=(0.0, 0.006, 0.044)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.0, lower=-0.06, upper=0.06),
    )
    model.articulation(
        "right_hinge",
        ArticulationType.REVOLUTE,
        parent=bridge,
        child=right_body,
        origin=Origin(xyz=(0.0, 0.006, 0.044)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.0, lower=-0.06, upper=0.06),
    )
    model.articulation(
        "focus_spin",
        ArticulationType.CONTINUOUS,
        parent=bridge,
        child=focus_knob,
        origin=Origin(xyz=(0.0, -0.030, 0.092)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
    )
    model.articulation(
        "left_diopter_spin",
        ArticulationType.CONTINUOUS,
        parent=left_body,
        child=left_diopter,
        origin=Origin(xyz=(0.026, -0.074, 0.014)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=10.0),
    )
    model.articulation(
        "right_diopter_spin",
        ArticulationType.CONTINUOUS,
        parent=right_body,
        child=right_diopter,
        origin=Origin(xyz=(-0.026, -0.074, 0.014)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.15, velocity=10.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bridge = object_model.get_part("bridge")
    left_body = object_model.get_part("left_body")
    right_body = object_model.get_part("right_body")
    focus_knob = object_model.get_part("focus_knob")
    left_diopter = object_model.get_part("left_diopter")
    right_diopter = object_model.get_part("right_diopter")

    left_hinge = object_model.get_articulation("left_hinge")
    right_hinge = object_model.get_articulation("right_hinge")
    focus_spin = object_model.get_articulation("focus_spin")
    left_diopter_spin = object_model.get_articulation("left_diopter_spin")
    right_diopter_spin = object_model.get_articulation("right_diopter_spin")

    hinge_pin = bridge.get_visual("hinge_pin")
    left_focus_shaft = bridge.get_visual("left_focus_shaft")
    right_focus_shaft = bridge.get_visual("right_focus_shaft")

    left_hinge_barrel = left_body.get_visual("hinge_barrel")
    right_hinge_barrel = right_body.get_visual("hinge_barrel")
    left_objective = left_body.get_visual("objective_barrel")
    right_objective = right_body.get_visual("objective_barrel")
    left_eyepiece = left_body.get_visual("eyepiece_tube")
    right_eyepiece = right_body.get_visual("eyepiece_tube")
    left_eyepiece_bridge = left_body.get_visual("eyepiece_bridge")
    right_eyepiece_bridge = right_body.get_visual("eyepiece_bridge")
    left_diopter_seat = left_body.get_visual("diopter_seat")
    right_diopter_seat = right_body.get_visual("diopter_seat")
    focus_core = focus_knob.get_visual("focus_core")
    focus_wheel = focus_knob.get_visual("focus_wheel")
    left_ring_core = left_diopter.get_visual("ring_core")
    right_ring_core = right_diopter.get_visual("ring_core")

    ctx.allow_overlap(left_body, bridge, reason="left hinge barrel sleeves around the central hinge pin")
    ctx.allow_overlap(right_body, bridge, reason="right hinge barrel sleeves around the central hinge pin")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    ctx.warn_if_part_geometry_disconnected()
    ctx.check_articulation_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )
    ctx.warn_if_overlaps(
        max_pose_samples=128,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_overlap(left_body, bridge, axes="yz", min_overlap=0.009, elem_a=left_hinge_barrel, elem_b=hinge_pin)
    ctx.expect_overlap(right_body, bridge, axes="yz", min_overlap=0.009, elem_a=right_hinge_barrel, elem_b=hinge_pin)

    ctx.expect_overlap(focus_knob, bridge, axes="yz", min_overlap=0.009, elem_a=focus_core, elem_b=left_focus_shaft)
    ctx.expect_overlap(focus_knob, bridge, axes="yz", min_overlap=0.009, elem_a=focus_core, elem_b=right_focus_shaft)
    ctx.expect_contact(focus_knob, bridge, elem_a=focus_core, elem_b=left_focus_shaft)
    ctx.expect_contact(focus_knob, bridge, elem_a=focus_core, elem_b=right_focus_shaft)
    ctx.expect_gap(
        left_body,
        focus_knob,
        axis="x",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem=left_eyepiece,
        negative_elem=focus_wheel,
        name="focus_knob_sits_just_inboard_of_left_eyepiece",
    )
    ctx.expect_gap(
        focus_knob,
        right_body,
        axis="x",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem=focus_wheel,
        negative_elem=right_eyepiece,
        name="focus_knob_sits_just_inboard_of_right_eyepiece",
    )
    ctx.expect_gap(
        focus_knob,
        left_body,
        axis="z",
        min_gap=0.001,
        positive_elem=focus_wheel,
        negative_elem=left_eyepiece_bridge,
        name="focus_knob_rises_above_left_eyepiece_bridge",
    )
    ctx.expect_gap(
        focus_knob,
        right_body,
        axis="z",
        min_gap=0.001,
        positive_elem=focus_wheel,
        negative_elem=right_eyepiece_bridge,
        name="focus_knob_rises_above_right_eyepiece_bridge",
    )

    ctx.expect_gap(
        left_body,
        right_body,
        axis="x",
        min_gap=0.090,
        positive_elem=left_objective,
        negative_elem=right_objective,
        name="front_objective_housings_are_wide_set",
    )
    ctx.expect_gap(
        left_body,
        right_body,
        axis="x",
        max_gap=0.020,
        max_penetration=0.0,
        positive_elem=left_eyepiece,
        negative_elem=right_eyepiece,
        name="rear_eyepiece_housings_are_much_closer",
    )
    ctx.expect_gap(
        left_body,
        left_body,
        axis="x",
        min_gap=0.012,
        positive_elem=left_objective,
        negative_elem=left_eyepiece,
        name="left_objective_sits_outboard_of_left_eyepiece",
    )
    ctx.expect_gap(
        right_body,
        right_body,
        axis="x",
        min_gap=0.012,
        positive_elem=right_eyepiece,
        negative_elem=right_objective,
        name="right_objective_sits_outboard_of_right_eyepiece",
    )

    ctx.expect_overlap(left_diopter, left_body, axes="xz", min_overlap=0.030, elem_a=left_ring_core, elem_b=left_diopter_seat)
    ctx.expect_contact(left_diopter, left_body, elem_a=left_ring_core, elem_b=left_diopter_seat)
    ctx.expect_overlap(right_diopter, right_body, axes="xz", min_overlap=0.030, elem_a=right_ring_core, elem_b=right_diopter_seat)
    ctx.expect_contact(right_diopter, right_body, elem_a=right_ring_core, elem_b=right_diopter_seat)

    with ctx.pose(
        {
            left_hinge: -0.06,
            right_hinge: 0.06,
            focus_spin: 1.4,
            left_diopter_spin: 0.8,
            right_diopter_spin: -0.8,
        }
    ):
        ctx.expect_gap(
            left_body,
            right_body,
            axis="x",
            min_gap=0.116,
            positive_elem=left_objective,
            negative_elem=right_objective,
            name="objective_spacing_opens_further_in_wide_pose",
        )
        ctx.expect_gap(
            left_body,
            right_body,
            axis="x",
            min_gap=0.010,
            positive_elem=left_eyepiece,
            negative_elem=right_eyepiece,
            name="eyepieces_remain_clear_when_hinge_opens",
        )
        ctx.expect_contact(focus_knob, bridge, elem_a=focus_core, elem_b=left_focus_shaft)
        ctx.expect_contact(focus_knob, bridge, elem_a=focus_core, elem_b=right_focus_shaft)
        ctx.expect_contact(left_diopter, left_body, elem_a=left_ring_core, elem_b=left_diopter_seat)
        ctx.expect_contact(right_diopter, right_body, elem_a=right_ring_core, elem_b=right_diopter_seat)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
