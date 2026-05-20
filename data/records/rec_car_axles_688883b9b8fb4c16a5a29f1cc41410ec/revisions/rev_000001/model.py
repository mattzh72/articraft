from __future__ import annotations

import math

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


def _hub_shell_mesh(name: str):
    """One wheel-hub shell, authored with its mounting face at local x=0."""

    gap = 0.0

    def annular_x(x0: float, length: float, outer_radius: float, bore_radius: float):
        return (
            cq.Workplane("YZ", origin=(x0, 0.0, 0.0))
            .circle(outer_radius)
            .circle(bore_radius)
            .extrude(length)
        )

    # Rear bearing sleeve and broad wheel-mounting flange share the same bore.
    hub = annular_x(gap, 0.108, 0.055, 0.033)
    hub = hub.union(annular_x(gap, 0.045, 0.118, 0.033))

    # A closed grease cap sits beyond the hidden spindle nose, leaving the rear
    # half of the hub as a true clearance bore rather than a solid placeholder.
    hub = hub.union(
        cq.Workplane("YZ", origin=(0.087, 0.0, 0.0)).circle(0.041).extrude(0.028)
    )

    # Five protruding wheel studs make hub spin visually legible.
    stud_radius = 0.008
    stud_circle = 0.079
    for index in range(5):
        angle = 2.0 * math.pi * index / 5.0 + math.radians(18.0)
        y = stud_circle * math.cos(angle)
        z = stud_circle * math.sin(angle)
        stud = (
            cq.Workplane("YZ", origin=(0.040, y, z))
            .circle(stud_radius)
            .extrude(0.041)
        )
        hub = hub.union(stud)

    # A shallow pilot ring at the center of the flange gives the hub a machined
    # bearing-seat silhouette while preserving the spindle bore behind it.
    hub = hub.union(
        cq.Workplane("YZ", origin=(0.045, 0.0, 0.0))
        .circle(0.065)
        .circle(0.035)
        .extrude(0.026)
    )

    return mesh_from_cadquery(hub, name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="de_dion_rear_axle")

    tube_paint = model.material("satin_black_tube", color=(0.02, 0.022, 0.024, 1.0))
    dark_block = model.material("dark_forged_block", color=(0.07, 0.075, 0.078, 1.0))
    machined = model.material("machined_steel", color=(0.62, 0.60, 0.56, 1.0))
    pin_steel = model.material("polished_pin_steel", color=(0.78, 0.76, 0.70, 1.0))

    axle_beam = model.part("axle_beam")

    # Two visible tube halves terminate in the center universal-coupling block,
    # just like a de Dion tube with the center section clamped/welded in place.
    axle_beam.visual(
        Cylinder(radius=0.055, length=0.84),
        origin=Origin(xyz=(-0.51, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tube_paint,
        name="left_tube",
    )
    axle_beam.visual(
        Cylinder(radius=0.055, length=0.84),
        origin=Origin(xyz=(0.51, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tube_paint,
        name="right_tube",
    )

    axle_beam.visual(
        Box((0.235, 0.185, 0.150)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_block,
        name="coupling_block",
    )
    axle_beam.visual(
        Box((0.075, 0.290, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=dark_block,
        name="upper_yoke",
    )
    axle_beam.visual(
        Box((0.075, 0.290, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=dark_block,
        name="lower_yoke",
    )
    axle_beam.visual(
        Cylinder(radius=0.018, length=0.330),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=pin_steel,
        name="cross_pin_y",
    )
    axle_beam.visual(
        Cylinder(radius=0.015, length=0.185),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=pin_steel,
        name="cross_pin_z",
    )

    # Outer forged end flanges and hidden stub spindles.  The thin shafts pass
    # through the hub bores; the hub joint origins sit on the flange faces.
    flange_face_x = 0.940
    flange_thickness = 0.055
    axle_beam.visual(
        Cylinder(radius=0.080, length=flange_thickness),
        origin=Origin(
            xyz=(flange_face_x - flange_thickness / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=machined,
        name="right_spindle_flange",
    )
    axle_beam.visual(
        Cylinder(radius=0.044, length=0.075),
        origin=Origin(
            xyz=(flange_face_x - flange_thickness - 0.030, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=machined,
        name="right_neck",
    )
    axle_beam.visual(
        Cylinder(radius=0.024, length=0.074),
        origin=Origin(
            xyz=(flange_face_x + 0.074 / 2.0, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=pin_steel,
        name="right_spindle_shaft",
    )
    axle_beam.visual(
        Cylinder(radius=0.080, length=flange_thickness),
        origin=Origin(
            xyz=(-(flange_face_x - flange_thickness / 2.0), 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=machined,
        name="left_spindle_flange",
    )
    axle_beam.visual(
        Cylinder(radius=0.044, length=0.075),
        origin=Origin(
            xyz=(-(flange_face_x - flange_thickness - 0.030), 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=machined,
        name="left_neck",
    )
    axle_beam.visual(
        Cylinder(radius=0.024, length=0.074),
        origin=Origin(
            xyz=(-(flange_face_x + 0.074 / 2.0), 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=pin_steel,
        name="left_spindle_shaft",
    )

    right_hub = model.part("right_hub")
    right_hub.visual(
        _hub_shell_mesh("right_hub_shell"),
        origin=Origin(),
        material=machined,
        name="hub_shell",
    )

    left_hub = model.part("left_hub")
    left_hub.visual(
        _hub_shell_mesh("left_hub_shell"),
        origin=Origin(),
        material=machined,
        name="hub_shell",
    )

    model.articulation(
        "right_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=axle_beam,
        child=right_hub,
        origin=Origin(xyz=(flange_face_x, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=55.0),
    )
    model.articulation(
        "left_hub_spin",
        ArticulationType.CONTINUOUS,
        parent=axle_beam,
        child=left_hub,
        # Rotate the joint frame so the same local hub mesh projects outward.
        origin=Origin(xyz=(-flange_face_x, 0.0, 0.0), rpy=(0.0, 0.0, math.pi)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=55.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    axle_beam = object_model.get_part("axle_beam")
    right_hub = object_model.get_part("right_hub")
    left_hub = object_model.get_part("left_hub")
    right_spin = object_model.get_articulation("right_hub_spin")
    left_spin = object_model.get_articulation("left_hub_spin")

    ctx.check(
        "hub joints are continuous revolutes",
        right_spin.articulation_type == ArticulationType.CONTINUOUS
        and left_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"right={right_spin.articulation_type}, left={left_spin.articulation_type}",
    )
    ctx.check(
        "hub joints spin about spindle axes",
        right_spin.axis == (1.0, 0.0, 0.0) and left_spin.axis == (1.0, 0.0, 0.0),
        details=f"right={right_spin.axis}, left={left_spin.axis}",
    )

    ctx.expect_gap(
        right_hub,
        axle_beam,
        axis="x",
        positive_elem="hub_shell",
        negative_elem="right_spindle_flange",
        min_gap=0.0,
        max_gap=0.0005,
        name="right hub seats at flange face",
    )
    ctx.expect_gap(
        axle_beam,
        left_hub,
        axis="x",
        positive_elem="left_spindle_flange",
        negative_elem="hub_shell",
        min_gap=0.0,
        max_gap=0.0005,
        name="left hub seats at flange face",
    )
    ctx.expect_overlap(
        right_hub,
        axle_beam,
        axes="x",
        elem_a="hub_shell",
        elem_b="right_spindle_shaft",
        min_overlap=0.060,
        name="right spindle projects into hub bore",
    )
    ctx.expect_overlap(
        left_hub,
        axle_beam,
        axes="x",
        elem_a="hub_shell",
        elem_b="left_spindle_shaft",
        min_overlap=0.060,
        name="left spindle projects into hub bore",
    )

    return ctx.report()


object_model = build_object_model()
