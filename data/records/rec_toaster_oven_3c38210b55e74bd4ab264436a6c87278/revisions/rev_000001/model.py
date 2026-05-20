from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="calibration_toaster_oven")

    brushed = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    dark = model.material("black_shadow_gap", rgba=(0.015, 0.014, 0.012, 1.0))
    panel = model.material("machined_control_panel", rgba=(0.50, 0.52, 0.51, 1.0))
    black = model.material("matte_black", rgba=(0.04, 0.04, 0.038, 1.0))
    glass = model.material("smoked_glass", rgba=(0.10, 0.16, 0.18, 0.45))
    steel = model.material("ground_steel", rgba=(0.82, 0.82, 0.78, 1.0))
    amber = model.material("warm_heater", rgba=(1.0, 0.32, 0.05, 1.0))
    ink = model.material("black_calibration_ink", rgba=(0.0, 0.0, 0.0, 1.0))

    body = model.part("body")

    # Rectangular oven shell with a real front opening.  The right-side tower is
    # a continuous control bay; the left bay remains hollow for the oven cavity.
    body.visual(Box((0.55, 0.38, 0.025)), origin=Origin(xyz=(0.0, 0.0, 0.3075)), material=brushed, name="top_shell")
    body.visual(Box((0.55, 0.38, 0.025)), origin=Origin(xyz=(0.0, 0.0, 0.0125)), material=brushed, name="bottom_shell")
    body.visual(Box((0.025, 0.38, 0.32)), origin=Origin(xyz=(-0.2625, 0.0, 0.16)), material=brushed, name="side_wall")
    body.visual(Box((0.025, 0.38, 0.32)), origin=Origin(xyz=(0.1225, 0.0, 0.16)), material=brushed, name="control_divider")
    body.visual(Box((0.025, 0.38, 0.32)), origin=Origin(xyz=(0.2625, 0.0, 0.16)), material=brushed, name="control_side_wall")
    body.visual(Box((0.55, 0.025, 0.32)), origin=Origin(xyz=(0.0, 0.1775, 0.16)), material=brushed, name="rear_wall")
    body.visual(Box((0.13, 0.018, 0.28)), origin=Origin(xyz=(0.195, -0.196, 0.16)), material=panel, name="control_face")

    # Finished front frame and controlled shadow gap around the door perimeter.
    body.visual(Box((0.385, 0.016, 0.030)), origin=Origin(xyz=(-0.070, -0.198, 0.292)), material=brushed, name="front_lintel")
    body.visual(Box((0.385, 0.016, 0.030)), origin=Origin(xyz=(-0.070, -0.198, 0.045)), material=brushed, name="front_sill")
    body.visual(Box((0.030, 0.016, 0.270)), origin=Origin(xyz=(-0.247, -0.198, 0.168)), material=brushed, name="front_jamb")
    body.visual(Box((0.030, 0.016, 0.270)), origin=Origin(xyz=(0.107, -0.198, 0.168)), material=brushed, name="control_jamb")
    body.visual(Box((0.342, 0.004, 0.008)), origin=Origin(xyz=(-0.070, -0.208, 0.276)), material=dark, name="top_gap_gauge")
    body.visual(Box((0.342, 0.004, 0.008)), origin=Origin(xyz=(-0.070, -0.208, 0.062)), material=dark, name="bottom_gap_gauge")
    body.visual(Box((0.008, 0.004, 0.214)), origin=Origin(xyz=(-0.232, -0.208, 0.169)), material=dark, name="side_gap_gauge")
    body.visual(Box((0.008, 0.004, 0.214)), origin=Origin(xyz=(0.092, -0.208, 0.169)), material=dark, name="divider_gap_gauge")

    # Interior: dark back, rack ledges, and exposed heater rods tied to the side
    # walls so the cavity reads as a manufacturable oven rather than a black box.
    body.visual(Box((0.345, 0.006, 0.225)), origin=Origin(xyz=(-0.070, 0.162, 0.165)), material=black, name="cavity_back")
    for z in (0.115, 0.205):
        body.visual(Box((0.010, 0.250, 0.010)), origin=Origin(xyz=(-0.245, -0.020, z)), material=steel, name=f"rack_rail_{z:.3f}_0")
        body.visual(Box((0.010, 0.250, 0.010)), origin=Origin(xyz=(0.105, -0.020, z)), material=steel, name=f"rack_rail_{z:.3f}_1")
    for z in (0.075, 0.245):
        body.visual(Cylinder(radius=0.006, length=0.360), origin=Origin(xyz=(-0.070, -0.030, z), rpy=(0.0, pi / 2.0, 0.0)), material=amber, name=f"heater_rod_{z:.3f}")

    top_vent = SlotPatternPanelGeometry(
        (0.250, 0.095),
        0.004,
        slot_size=(0.030, 0.005),
        pitch=(0.042, 0.017),
        frame=0.012,
        corner_radius=0.004,
        stagger=True,
    )
    body.visual(
        mesh_from_geometry(top_vent, "top_vent_slots"),
        origin=Origin(xyz=(-0.065, -0.010, 0.322), rpy=(0.0, 0.0, 0.0)),
        material=dark,
        name="top_vent_slots",
    )

    # Alternating hinge knuckles and a leaf welded to the front sill.  Door
    # knuckles fill the gaps but do not collide with these stationary barrels.
    for i, x in enumerate((-0.180, 0.000, 0.180)):
        barrel_length = 0.040 if abs(x) < 1e-9 else 0.080
        body.visual(Cylinder(radius=0.012, length=barrel_length), origin=Origin(xyz=(x, -0.225, 0.045), rpy=(0.0, pi / 2.0, 0.0)), material=steel, name=f"body_hinge_barrel_{i}")
        leaf_width = 0.032 if abs(x) < 1e-9 else 0.060
        body.visual(Box((leaf_width, 0.032, 0.032)), origin=Origin(xyz=(x, -0.209, 0.061)), material=steel, name=f"body_hinge_leaf_{i}")

    # Datum-friendly control bay: flat pads, bushings, index marks, and trim
    # screws are all mounted to the same front panel.
    knob_specs = (
        ("thermostat", 0.235),
        ("function", 0.160),
        ("timer", 0.085),
    )
    for label, z in knob_specs:
        body.visual(Box((0.096, 0.003, 0.056)), origin=Origin(xyz=(0.215, -0.2065, z)), material=steel, name=f"{label}_datum_pad")
        body.visual(Cylinder(radius=0.026, length=0.012), origin=Origin(xyz=(0.215, -0.211, z), rpy=(pi / 2.0, 0.0, 0.0)), material=steel, name=f"{label}_shaft_bushing")
        body.visual(Box((0.004, 0.002, 0.026)), origin=Origin(xyz=(0.215, -0.209, z + 0.027)), material=ink, name=f"{label}_zero_mark")
        body.visual(Box((0.018, 0.002, 0.004)), origin=Origin(xyz=(0.178, -0.209, z)), material=ink, name=f"{label}_low_mark")
        body.visual(Box((0.018, 0.002, 0.004)), origin=Origin(xyz=(0.252, -0.209, z)), material=ink, name=f"{label}_high_mark")
    for i, z in enumerate((0.271, 0.049)):
        body.visual(Cylinder(radius=0.008, length=0.004), origin=Origin(xyz=(0.245, -0.207, z), rpy=(pi / 2.0, 0.0, 0.0)), material=steel, name=f"trim_screw_head_{i}")
        body.visual(Box((0.012, 0.002, 0.0025)), origin=Origin(xyz=(0.245, -0.210, z)), material=ink, name=f"trim_screw_slot_{i}")

    # Four low feet are fused to the bottom shell and establish a stable datum.
    for i, (x, y) in enumerate(((-0.215, -0.125), (0.215, -0.125), (-0.215, 0.125), (0.215, 0.125))):
        body.visual(Box((0.060, 0.045, 0.018)), origin=Origin(xyz=(x, y, -0.009)), material=black, name=f"foot_{i}")

    door = model.part("door")
    # The child frame is the bottom hinge axis.  The door panel extends in +Z;
    # positive rotation about +X swings the top edge forward and downward.
    for i, x in enumerate((-0.080, 0.080)):
        door.visual(Cylinder(radius=0.012, length=0.120), origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)), material=steel, name=f"door_hinge_barrel_{i}")
        door.visual(Box((0.085, 0.010, 0.040)), origin=Origin(xyz=(x, -0.006, 0.025)), material=steel, name=f"door_hinge_leaf_{i}")
    door.visual(Box((0.355, 0.020, 0.030)), origin=Origin(xyz=(0.0, -0.015, 0.050)), material=brushed, name="door_bottom_rail")
    door.visual(Box((0.355, 0.020, 0.030)), origin=Origin(xyz=(0.0, -0.015, 0.250)), material=brushed, name="door_top_rail")
    door.visual(Box((0.030, 0.020, 0.220)), origin=Origin(xyz=(-0.1625, -0.015, 0.150)), material=brushed, name="door_side_rail_0")
    door.visual(Box((0.030, 0.020, 0.220)), origin=Origin(xyz=(0.1625, -0.015, 0.150)), material=brushed, name="door_side_rail_1")
    door.visual(Box((0.300, 0.006, 0.175)), origin=Origin(xyz=(0.0, -0.026, 0.152)), material=glass, name="glass_window")
    door.visual(Box((0.270, 0.024, 0.023)), origin=Origin(xyz=(0.0, -0.062, 0.205)), material=black, name="door_handle")
    for x in (-0.105, 0.105):
        door.visual(Box((0.018, 0.048, 0.030)), origin=Origin(xyz=(x, -0.041, 0.205)), material=black, name=f"handle_standoff_{x:+.3f}")

    door_hinge = model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, -0.225, 0.045)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5, lower=0.0, upper=1.35),
    )
    door_hinge.meta["qc_samples"] = [0.0, 0.65, 1.25]

    knob_meshes = {
        "thermostat_knob": mesh_from_geometry(
            KnobGeometry(
                0.048,
                0.030,
                body_style="faceted",
                base_diameter=0.052,
                top_diameter=0.038,
                edge_radius=0.0010,
                grip=KnobGrip(style="ribbed", count=24, depth=0.0010, width=0.0015),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
                center=False,
            ),
            "thermostat_knob_mesh",
        ),
        "function_knob": mesh_from_geometry(
            KnobGeometry(
                0.044,
                0.028,
                body_style="faceted",
                base_diameter=0.049,
                top_diameter=0.036,
                edge_radius=0.0010,
                grip=KnobGrip(style="ribbed", count=18, depth=0.0010, width=0.0016),
                indicator=KnobIndicator(style="wedge", mode="raised", angle_deg=0.0),
                bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
                center=False,
            ),
            "function_knob_mesh",
        ),
        "timer_knob": mesh_from_geometry(
            KnobGeometry(
                0.048,
                0.030,
                body_style="faceted",
                base_diameter=0.052,
                top_diameter=0.038,
                edge_radius=0.0010,
                grip=KnobGrip(style="ribbed", count=30, depth=0.0010, width=0.0013),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
                center=False,
            ),
            "timer_knob_mesh",
        ),
    }

    for part_name, z in (
        ("thermostat_knob", 0.235),
        ("function_knob", 0.160),
        ("timer_knob", 0.085),
    ):
        knob = model.part(part_name)
        knob.visual(Cylinder(radius=0.006, length=0.014), origin=Origin(xyz=(0.0, 0.0, 0.007)), material=steel, name="drive_shaft")
        knob.visual(knob_meshes[part_name], origin=Origin(xyz=(0.0, 0.0, 0.014)), material=black, name="calibration_cap")
        joint = model.articulation(
            f"body_to_{part_name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(0.215, -0.217, z), rpy=(pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.6, velocity=4.0, lower=-2.35, upper=2.35),
        )
        joint.meta["qc_samples"] = [-1.2, 0.0, 1.2]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("body_to_door")

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            min_gap=0.010,
            max_gap=0.035,
            positive_elem="top_gap_gauge",
            negative_elem="door_top_rail",
            name="closed door has controlled front gap",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a="glass_window",
            elem_b="cavity_back",
            min_overlap=0.10,
            name="door window aligns with oven cavity",
        )

    closed_top = ctx.part_element_world_aabb(door, elem="door_top_rail")
    with ctx.pose({door_hinge: 1.10}):
        opened_top = ctx.part_element_world_aabb(door, elem="door_top_rail")
    ctx.check(
        "door hinge swings top edge forward and downward",
        closed_top is not None
        and opened_top is not None
        and opened_top[0][1] < closed_top[0][1] - 0.10
        and opened_top[1][2] < closed_top[1][2] - 0.08,
        details=f"closed={closed_top}, opened={opened_top}",
    )

    for part_name, bushing in (
        ("thermostat_knob", "thermostat_shaft_bushing"),
        ("function_knob", "function_shaft_bushing"),
        ("timer_knob", "timer_shaft_bushing"),
    ):
        knob = object_model.get_part(part_name)
        shaft_joint = object_model.get_articulation(f"body_to_{part_name}")
        ctx.expect_contact(
            knob,
            body,
            elem_a="drive_shaft",
            elem_b=bushing,
            contact_tol=0.001,
            name=f"{part_name} shaft seats against its bushing",
        )
        before = ctx.part_element_world_aabb(knob, elem="calibration_cap")
        with ctx.pose({shaft_joint: 1.0}):
            after = ctx.part_element_world_aabb(knob, elem="calibration_cap")
        ctx.check(
            f"{part_name} rotates on panel shaft",
            before is not None and after is not None and abs(after[0][1] - before[0][1]) < 0.003,
            details=f"before={before}, after={after}",
        )

    return ctx.report()


object_model = build_object_model()
