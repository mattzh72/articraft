from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_desktop_toaster_oven")

    brushed = model.material("brushed_stainless", rgba=(0.70, 0.72, 0.70, 1.0))
    dark = model.material("matte_black", rgba=(0.02, 0.022, 0.024, 1.0))
    inner = model.material("dark_cavity", rgba=(0.09, 0.085, 0.075, 1.0))
    glass = model.material("smoked_glass", rgba=(0.05, 0.07, 0.08, 0.42))
    chrome = model.material("polished_chrome", rgba=(0.86, 0.88, 0.86, 1.0))
    orange = model.material("warm_heater", rgba=(1.0, 0.42, 0.08, 1.0))
    white = model.material("printed_white", rgba=(0.92, 0.90, 0.82, 1.0))
    rubber = model.material("soft_feet", rgba=(0.012, 0.012, 0.012, 1.0))

    body = model.part("body")

    # Efficient shoebox envelope: thin structural shell around a real front opening.
    body.visual(Box((0.320, 0.440, 0.018)), origin=Origin(xyz=(0.000, 0.000, 0.055)), material=brushed, name="bottom_shell")
    body.visual(Box((0.320, 0.440, 0.018)), origin=Origin(xyz=(0.000, 0.000, 0.245)), material=brushed, name="top_shell")
    body.visual(Box((0.320, 0.018, 0.190)), origin=Origin(xyz=(0.000, -0.211, 0.150)), material=brushed, name="side_wall")
    body.visual(Box((0.320, 0.018, 0.190)), origin=Origin(xyz=(0.000, 0.211, 0.150)), material=brushed, name="control_wall")
    body.visual(Box((0.018, 0.440, 0.190)), origin=Origin(xyz=(0.151, 0.000, 0.150)), material=brushed, name="back_wall")
    body.visual(Box((0.320, 0.016, 0.190)), origin=Origin(xyz=(0.000, 0.119, 0.150)), material=inner, name="divider_wall")
    body.visual(Box((0.014, 0.094, 0.190)), origin=Origin(xyz=(-0.165, 0.165, 0.150)), material=dark, name="control_panel")

    # Front lips frame the cooking cavity without filling it.
    body.visual(Box((0.014, 0.334, 0.018)), origin=Origin(xyz=(-0.165, -0.046, 0.075)), material=brushed, name="lower_lip")
    body.visual(Box((0.014, 0.334, 0.018)), origin=Origin(xyz=(-0.165, -0.046, 0.225)), material=brushed, name="upper_lip")
    body.visual(Box((0.014, 0.018, 0.150)), origin=Origin(xyz=(-0.165, -0.211, 0.150)), material=brushed, name="front_side_lip")
    body.visual(Box((0.014, 0.018, 0.150)), origin=Origin(xyz=(-0.165, 0.119, 0.150)), material=brushed, name="front_divider_lip")

    # Interior elements are tied into the side walls and read through the glass.
    for z in (0.100, 0.205):
        body.visual(
            Cylinder(radius=0.0045, length=0.334),
            origin=Origin(xyz=(-0.018, -0.046, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=orange,
            name=f"heater_{z:.3f}",
        )
    for x in (-0.095, -0.020, 0.055):
        body.visual(
            Cylinder(radius=0.0022, length=0.334),
            origin=Origin(xyz=(x, -0.046, 0.148), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"rack_wire_{x:.3f}",
        )

    # Segmented exposed hinge barrels on the body side of the bottom hinge.
    body.visual(
        Cylinder(radius=0.003, length=0.370),
        origin=Origin(xyz=(-0.183, -0.046, 0.076), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_pin",
    )
    for idx, (y, length) in enumerate(((-0.219, 0.026), (0.132, 0.026))):
        body.visual(
            Cylinder(radius=0.007, length=length),
            origin=Origin(xyz=(-0.183, y, 0.076), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"fixed_hinge_{idx}",
        )
        body.visual(
            Box((0.020, length + 0.002, 0.008)),
            origin=Origin(xyz=(-0.173, y, 0.069)),
            material=brushed,
            name=f"hinge_leaf_{idx}",
        )

    # Three compact shaft bearings on a narrow control column.
    knob_centers = {
        "temp": 0.205,
        "mode": 0.155,
        "timer": 0.105,
    }
    for name, z in knob_centers.items():
        body.visual(
            Cylinder(radius=0.021, length=0.010),
            origin=Origin(xyz=(-0.177, 0.165, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name=f"{name}_collar",
        )
        # Raised printed ticks are thin but physically seated on the panel.
        body.visual(Box((0.004, 0.006, 0.0025)), origin=Origin(xyz=(-0.174, 0.165, z + 0.032)), material=white, name=f"{name}_tick_top")
        body.visual(Box((0.004, 0.006, 0.0025)), origin=Origin(xyz=(-0.174, 0.165, z - 0.032)), material=white, name=f"{name}_tick_bottom")
        body.visual(Box((0.004, 0.0025, 0.006)), origin=Origin(xyz=(-0.174, 0.134, z)), material=white, name=f"{name}_tick_low")
        body.visual(Box((0.004, 0.0025, 0.006)), origin=Origin(xyz=(-0.174, 0.196, z)), material=white, name=f"{name}_tick_high")

    # Rubber feet are recessed into the bottom shell, so the countertop footprint stays tight.
    for idx, (x, y) in enumerate(((-0.120, -0.160), (0.120, -0.160), (-0.120, 0.170), (0.120, 0.170))):
        body.visual(Box((0.045, 0.035, 0.012)), origin=Origin(xyz=(x, y, 0.041)), material=rubber, name=f"foot_{idx}")

    door = model.part("door")
    door.visual(Box((0.014, 0.315, 0.030)), origin=Origin(xyz=(0.000, 0.000, 0.018)), material=brushed, name="bottom_rail")
    door.visual(Box((0.014, 0.315, 0.024)), origin=Origin(xyz=(0.000, 0.000, 0.153)), material=brushed, name="top_rail")
    door.visual(Box((0.014, 0.026, 0.140)), origin=Origin(xyz=(0.000, -0.144, 0.083)), material=brushed, name="side_stile_0")
    door.visual(Box((0.014, 0.026, 0.140)), origin=Origin(xyz=(0.000, 0.144, 0.083)), material=brushed, name="side_stile_1")
    door.visual(Box((0.006, 0.250, 0.112)), origin=Origin(xyz=(-0.002, 0.000, 0.088)), material=glass, name="glass_pane")
    for idx, (y, length) in enumerate(((-0.045, 0.070), (0.095, 0.050))):
        door.visual(
            Cylinder(radius=0.007, length=length),
            origin=Origin(xyz=(0.000, y, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"door_hinge_{idx}",
        )
        door.visual(Box((0.016, length - 0.004, 0.010)), origin=Origin(xyz=(0.000, y, 0.008)), material=brushed, name=f"door_hinge_leaf_{idx}")
    door.visual(Box((0.034, 0.018, 0.032)), origin=Origin(xyz=(-0.021, -0.080, 0.122)), material=brushed, name="handle_mount_0")
    door.visual(Box((0.034, 0.018, 0.032)), origin=Origin(xyz=(-0.021, 0.080, 0.122)), material=brushed, name="handle_mount_1")
    door.visual(
        Cylinder(radius=0.008, length=0.200),
        origin=Origin(xyz=(-0.037, 0.000, 0.122), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="handle_bar",
    )

    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.183, -0.046, 0.076)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.58),
    )

    knob_meshes = {
        "temp_knob": mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.028,
                body_style="skirted",
                top_diameter=0.032,
                skirt=KnobSkirt(0.046, 0.005, flare=0.06, chamfer=0.001),
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", depth=0.0007),
                bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
            ),
            "temp_knob_cap",
        ),
        "mode_knob": mesh_from_geometry(
            KnobGeometry(
                0.038,
                0.026,
                body_style="faceted",
                top_diameter=0.030,
                grip=KnobGrip(style="ribbed", count=12, depth=0.001),
                indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=15.0),
                bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
            ),
            "mode_knob_cap",
        ),
        "timer_knob": mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.028,
                body_style="skirted",
                top_diameter=0.032,
                skirt=KnobSkirt(0.046, 0.005, flare=0.06, chamfer=0.001),
                grip=KnobGrip(style="fluted", count=24, depth=0.0011),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=-20.0),
                bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
            ),
            "timer_knob_cap",
        ),
    }
    for part_name, z in (("temp_knob", 0.205), ("mode_knob", 0.155), ("timer_knob", 0.105)):
        knob = model.part(part_name)
        knob.visual(
            Cylinder(radius=0.006, length=0.022),
            origin=Origin(xyz=(-0.010, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=chrome,
            name="shaft",
        )
        knob.visual(
            knob_meshes[part_name],
            origin=Origin(xyz=(-0.033, 0.000, 0.000), rpy=(0.0, -math.pi / 2.0, 0.0)),
            material=dark,
            name="cap",
        )

    model.articulation(
        "temp_shaft",
        ArticulationType.REVOLUTE,
        parent=body,
        child="temp_knob",
        origin=Origin(xyz=(-0.182, 0.165, 0.205)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0, lower=-2.35, upper=2.35),
    )
    model.articulation(
        "mode_shaft",
        ArticulationType.REVOLUTE,
        parent=body,
        child="mode_knob",
        origin=Origin(xyz=(-0.182, 0.165, 0.155)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0, lower=-1.75, upper=1.75),
    )
    model.articulation(
        "timer_shaft",
        ArticulationType.REVOLUTE,
        parent=body,
        child="timer_knob",
        origin=Origin(xyz=(-0.182, 0.165, 0.105)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0, lower=0.0, upper=4.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("door_hinge")

    for hinge_elem in ("door_hinge_0", "door_hinge_1"):
        ctx.allow_overlap(
            body,
            door,
            elem_a="hinge_pin",
            elem_b=hinge_elem,
            reason="The stainless hinge pin intentionally passes through the rotating door knuckle.",
        )
        ctx.expect_within(
            body,
            door,
            axes="xz",
            inner_elem="hinge_pin",
            outer_elem=hinge_elem,
            margin=0.0005,
            name=f"hinge pin centered in {hinge_elem}",
        )
        ctx.expect_overlap(
            body,
            door,
            axes="y",
            elem_a="hinge_pin",
            elem_b=hinge_elem,
            min_overlap=0.040,
            name=f"hinge pin retained by {hinge_elem}",
        )

    for knob_name, collar_name in (
        ("temp_knob", "temp_collar"),
        ("mode_knob", "mode_collar"),
        ("timer_knob", "timer_collar"),
    ):
        knob = object_model.get_part(knob_name)
        ctx.allow_overlap(
            body,
            knob,
            elem_a=collar_name,
            elem_b="shaft",
            reason="The visible control shaft is intentionally captured inside the panel bearing collar.",
        )
        ctx.expect_within(
            knob,
            body,
            axes="yz",
            inner_elem="shaft",
            outer_elem=collar_name,
            margin=0.001,
            name=f"{knob_name} shaft centered in collar",
        )
        ctx.expect_overlap(
            knob,
            body,
            axes="x",
            elem_a="shaft",
            elem_b=collar_name,
            min_overlap=0.0004,
            name=f"{knob_name} shaft retained in collar",
        )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="x",
            positive_elem="lower_lip",
            negative_elem="bottom_rail",
            min_gap=0.003,
            max_gap=0.012,
            name="closed door clears front sill",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="y",
            elem_a="glass_pane",
            elem_b="lower_lip",
            min_overlap=0.240,
            name="door spans the cooking opening",
        )

    closed_top = ctx.part_element_world_aabb(door, elem="top_rail")
    with ctx.pose({door_hinge: 1.58}):
        folded_top = ctx.part_element_world_aabb(door, elem="top_rail")
        ctx.expect_gap(
            door,
            body,
            axis="z",
            positive_elem="glass_pane",
            negative_elem="bottom_shell",
            min_gap=0.002,
            name="folded door stays above counter shell",
        )
    ctx.check(
        "door folds flat forward",
        closed_top is not None
        and folded_top is not None
        and folded_top[1][2] < closed_top[1][2] - 0.055
        and folded_top[0][0] < closed_top[0][0] - 0.070,
        details=f"closed_top={closed_top}, folded_top={folded_top}",
    )

    return ctx.report()


object_model = build_object_model()
