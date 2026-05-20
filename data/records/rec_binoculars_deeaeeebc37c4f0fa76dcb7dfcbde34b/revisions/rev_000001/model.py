from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    LatheGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    mesh_from_geometry,
)


def _save_mesh(geometry: MeshGeometry, name: str):
    return mesh_from_geometry(geometry, name)


def _tube_mesh(*, outer_radius: float, inner_radius: float, length: float) -> MeshGeometry:
    outer = CylinderGeometry(outer_radius, length, radial_segments=56)
    inner = CylinderGeometry(inner_radius, length + 0.004, radial_segments=56)
    return boolean_difference(outer, inner)


def _barrel_mesh() -> MeshGeometry:
    """Stepped roof-prism barrel profile aligned to local Z before placement."""
    return LatheGeometry(
        [
            (0.0000, -0.073),
            (0.0140, -0.073),
            (0.0170, -0.070),
            (0.0170, -0.058),
            (0.0155, -0.054),
            (0.0155, -0.040),
            (0.0200, -0.034),
            (0.0200, 0.044),
            (0.0245, 0.050),
            (0.0255, 0.070),
            (0.0230, 0.074),
            (0.0000, 0.074),
        ],
        segments=72,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_prism_compact_binocular")

    armor = model.material("matte_rubber_armor", rgba=(0.025, 0.027, 0.028, 1.0))
    satin_black = model.material("satin_black", rgba=(0.006, 0.007, 0.008, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    glass = model.material("blue_green_lens_glass", rgba=(0.05, 0.22, 0.28, 0.82))
    marking_white = model.material("white_index_mark", rgba=(0.82, 0.84, 0.80, 1.0))

    barrel_mesh = _save_mesh(_barrel_mesh(), "stepped_roof_barrel")
    hinge_sleeve_mesh = _save_mesh(
        _tube_mesh(outer_radius=0.0095, inner_radius=0.0064, length=0.036),
        "right_hinge_sleeve",
    )
    diopter_mesh = _save_mesh(
        KnobGeometry(
            0.043,
            0.011,
            body_style="cylindrical",
            edge_radius=0.0006,
            grip=KnobGrip(style="ribbed", count=28, depth=0.0009, width=0.0013),
            bore=KnobBore(style="round", diameter=0.0338),
        ),
        "ribbed_diopter_ring",
    )
    focus_mesh = _save_mesh(
        KnobGeometry(
            0.032,
            0.024,
            body_style="hourglass",
            edge_radius=0.0008,
            grip=KnobGrip(style="fluted", count=24, depth=0.0012),
            bore=KnobBore(style="round", diameter=0.006),
        ),
        "fluted_focus_wheel",
    )

    left_body = model.part("left_body")
    left_body.visual(
        barrel_mesh,
        origin=Origin(xyz=(0.0, -0.034, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=armor,
        name="barrel_shell",
    )
    left_body.visual(
        Cylinder(radius=0.020, length=0.003),
        origin=Origin(xyz=(0.074, -0.034, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="objective_lens",
    )
    left_body.visual(
        Cylinder(radius=0.013, length=0.0025),
        origin=Origin(xyz=(-0.074, -0.034, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="eyepiece_lens",
    )
    left_body.visual(
        Box((0.078, 0.030, 0.012)),
        origin=Origin(xyz=(-0.032, -0.021, 0.021)),
        material=armor,
        name="bridge_arm",
    )
    left_body.visual(
        Cylinder(radius=0.0082, length=0.028),
        origin=Origin(xyz=(-0.026, 0.0, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=armor,
        name="hinge_lug",
    )
    left_body.visual(
        Cylinder(radius=0.0052, length=0.116),
        origin=Origin(xyz=(0.0, 0.0, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_pin",
    )
    left_body.visual(
        Box((0.030, 0.040, 0.006)),
        origin=Origin(xyz=(-0.045, 0.0, 0.031)),
        material=armor,
        name="focus_bridge_pad",
    )
    left_body.visual(
        Box((0.022, 0.004, 0.026)),
        origin=Origin(xyz=(-0.045, -0.016, 0.047)),
        material=armor,
        name="focus_yoke_0",
    )
    left_body.visual(
        Box((0.022, 0.004, 0.026)),
        origin=Origin(xyz=(-0.045, 0.016, 0.047)),
        material=armor,
        name="focus_yoke_1",
    )

    right_body = model.part("right_body")
    right_body.visual(
        barrel_mesh,
        origin=Origin(xyz=(0.0, 0.034, -0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=armor,
        name="barrel_shell",
    )
    right_body.visual(
        Cylinder(radius=0.020, length=0.003),
        origin=Origin(xyz=(0.074, 0.034, -0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="objective_lens",
    )
    right_body.visual(
        Cylinder(radius=0.013, length=0.0025),
        origin=Origin(xyz=(-0.074, 0.034, -0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="eyepiece_lens",
    )
    right_body.visual(
        hinge_sleeve_mesh,
        origin=Origin(xyz=(0.026, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=armor,
        name="hinge_sleeve",
    )
    right_body.visual(
        Box((0.086, 0.030, 0.012)),
        origin=Origin(xyz=(0.002, 0.019, -0.005)),
        material=armor,
        name="bridge_arm",
    )
    right_body.visual(
        Box((0.030, 0.004, 0.004)),
        origin=Origin(xyz=(-0.058, 0.034, -0.007)),
        material=marking_white,
        name="diopter_index",
    )

    focus_wheel = model.part("focus_wheel")
    focus_wheel.visual(
        focus_mesh,
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="focus_grip",
    )
    focus_wheel.visual(
        Cylinder(radius=0.0038, length=0.028),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="focus_axle",
    )

    diopter_ring = model.part("diopter_ring")
    diopter_ring.visual(
        diopter_mesh,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="diopter_grip",
    )
    diopter_ring.visual(
        Box((0.004, 0.0015, 0.010)),
        origin=Origin(xyz=(0.0, -0.0218, 0.0)),
        material=marking_white,
        name="diopter_mark",
    )

    model.articulation(
        "interpupillary_hinge",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=right_body,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=0.8, lower=-0.22, upper=0.22),
    )
    model.articulation(
        "center_focus",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=focus_wheel,
        origin=Origin(xyz=(-0.045, 0.0, 0.052)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=2.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "right_diopter",
        ArticulationType.REVOLUTE,
        parent=right_body,
        child=diopter_ring,
        origin=Origin(xyz=(-0.058, 0.034, -0.026)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=2.0, lower=-0.75, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    right_body = object_model.get_part("right_body")
    left_body = object_model.get_part("left_body")
    focus_wheel = object_model.get_part("focus_wheel")
    diopter_ring = object_model.get_part("diopter_ring")
    hinge = object_model.get_articulation("interpupillary_hinge")
    focus = object_model.get_articulation("center_focus")
    diopter = object_model.get_articulation("right_diopter")

    ctx.check(
        "primary controls are revolute",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and focus.articulation_type == ArticulationType.REVOLUTE
        and diopter.articulation_type == ArticulationType.REVOLUTE,
    )
    ctx.expect_overlap(
        right_body,
        left_body,
        axes="x",
        elem_a="hinge_sleeve",
        elem_b="hinge_pin",
        min_overlap=0.030,
        name="right hinge sleeve is captured on the central pin",
    )
    ctx.expect_gap(
        focus_wheel,
        left_body,
        axis="y",
        positive_elem="focus_axle",
        negative_elem="focus_yoke_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="focus axle seats against the bridge yoke",
    )
    ctx.allow_overlap(
        diopter_ring,
        right_body,
        elem_a="diopter_grip",
        elem_b="barrel_shell",
        reason="The diopter ring is intentionally a close rotating collar captured on the eyepiece bearing.",
    )
    ctx.expect_overlap(
        diopter_ring,
        right_body,
        axes="x",
        elem_a="diopter_grip",
        elem_b="barrel_shell",
        min_overlap=0.007,
        name="diopter ring encircles the right eyepiece barrel",
    )

    rest_aabb = ctx.part_element_world_aabb(right_body, elem="barrel_shell")
    with ctx.pose({hinge: hinge.motion_limits.upper}):
        wide_aabb = ctx.part_element_world_aabb(right_body, elem="barrel_shell")
    if rest_aabb is not None and wide_aabb is not None:
        rest_center_y = 0.5 * (rest_aabb[0][1] + rest_aabb[1][1])
        wide_center_y = 0.5 * (wide_aabb[0][1] + wide_aabb[1][1])
        ctx.check(
            "interpupillary hinge changes barrel spacing",
            wide_center_y > rest_center_y + 0.002,
            details=f"rest_y={rest_center_y:.4f}, wide_y={wide_center_y:.4f}",
        )
    else:
        ctx.fail("interpupillary hinge changes barrel spacing", "missing right barrel AABB")

    return ctx.report()


object_model = build_object_model()
