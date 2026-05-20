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
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


FRONT_Y = -0.245
HINGE_Y = -0.253
HINGE_Z = 0.135
HINGE_X = -0.075


def _visual(part, geometry, origin: Origin, material: Material, *, name: str) -> None:
    part.visual(geometry, origin=origin, material=material, name=name)


def _front_disk(part, name: str, xyz: tuple[float, float, float], radius: float, length: float, material: Material) -> None:
    """Thin cylinder normal to the front face, protruding toward -Y."""
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _x_cylinder(part, name: str, xyz: tuple[float, float, float], radius: float, length: float, material: Material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _y_cylinder(part, name: str, xyz: tuple[float, float, float], radius: float, length: float, material: Material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_toaster_oven")

    enamel = model.material("warm_cream_enamel", rgba=(0.78, 0.70, 0.56, 1.0))
    dark_enamel = model.material("dark_cavity_enamel", rgba=(0.035, 0.033, 0.030, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.62, 0.61, 0.57, 1.0))
    black = model.material("black_bakelite", rgba=(0.025, 0.023, 0.020, 1.0))
    glass = model.material("smoked_glass", rgba=(0.20, 0.32, 0.38, 0.42))
    brass = model.material("aged_brass", rgba=(0.78, 0.57, 0.30, 1.0))
    ceramic = model.material("ceramic_heater", rgba=(0.88, 0.72, 0.48, 1.0))
    red = model.material("red_indicator", rgba=(0.75, 0.07, 0.035, 1.0))

    body = model.part("oven_body")

    # Rectangular sheet-metal shell, assembled from overlapping panels so the
    # front stays open and visibly hollow.
    _visual(body, Box((0.60, 0.44, 0.025)), Origin(xyz=(0.0, 0.0, 0.4075)), enamel, name="top_shell")
    _visual(body, Box((0.60, 0.44, 0.025)), Origin(xyz=(0.0, 0.0, 0.0925)), enamel, name="bottom_shell")
    _visual(body, Box((0.025, 0.44, 0.34)), Origin(xyz=(-0.2875, 0.0, 0.25)), enamel, name="side_shell_0")
    _visual(body, Box((0.025, 0.44, 0.34)), Origin(xyz=(0.2875, 0.0, 0.25)), enamel, name="side_shell_1")
    _visual(body, Box((0.60, 0.025, 0.34)), Origin(xyz=(0.0, 0.2075, 0.25)), enamel, name="rear_shell")

    # Front flange and control bay: a legacy flat face bolted to the shell.
    _visual(body, Box((0.60, 0.025, 0.052)), Origin(xyz=(0.0, -0.2325, 0.109)), enamel, name="front_sill")
    _visual(body, Box((0.60, 0.025, 0.048)), Origin(xyz=(0.0, -0.2325, 0.396)), enamel, name="front_brow")
    _visual(body, Box((0.030, 0.030, 0.285)), Origin(xyz=(-0.272, -0.234, 0.255)), enamel, name="front_jamb_0")
    _visual(body, Box((0.030, 0.030, 0.285)), Origin(xyz=(0.130, -0.234, 0.255)), enamel, name="front_divider")
    _visual(body, Box((0.145, 0.030, 0.285)), Origin(xyz=(0.215, -0.234, 0.255)), brushed, name="control_panel")
    _visual(body, Box((0.390, 0.010, 0.215)), Origin(xyz=(-0.075, 0.190, 0.250)), dark_enamel, name="oven_cavity")

    # Practical reinforcement straps and corner angles.
    _visual(body, Box((0.028, 0.020, 0.340)), Origin(xyz=(-0.287, -0.238, 0.250)), brushed, name="corner_angle_0")
    _visual(body, Box((0.028, 0.020, 0.340)), Origin(xyz=(0.287, -0.238, 0.250)), brushed, name="corner_angle_1")
    _visual(body, Box((0.500, 0.018, 0.018)), Origin(xyz=(-0.050, -0.248, 0.411)), brushed, name="top_front_strap")
    _visual(body, Box((0.500, 0.018, 0.018)), Origin(xyz=(-0.050, -0.248, 0.095)), brushed, name="bottom_front_strap")
    for i, x in enumerate((-0.255, -0.155, -0.055, 0.045, 0.145, 0.195)):
        _front_disk(body, f"strap_bolt_{i}", (x, -0.257, 0.411), 0.006, 0.004, brass)
        _front_disk(body, f"sill_bolt_{i}", (x, -0.257, 0.095), 0.006, 0.004, brass)

    # Side and top service hatches, with proud screw heads.
    _visual(body, Box((0.205, 0.155, 0.006)), Origin(xyz=(-0.075, 0.030, 0.423)), brushed, name="top_service_hatch")
    for i, (x, y) in enumerate(((-0.165, -0.035), (0.015, -0.035), (-0.165, 0.095), (0.015, 0.095))):
        _visual(body, Cylinder(radius=0.006, length=0.006), Origin(xyz=(x, y, 0.4285)), brass, name=f"top_hatch_bolt_{i}")
    _visual(body, Box((0.006, 0.180, 0.120)), Origin(xyz=(-0.303, 0.028, 0.255)), brushed, name="side_service_hatch")
    for i, (y, z) in enumerate(((-0.045, 0.205), (0.100, 0.205), (-0.045, 0.305), (0.100, 0.305))):
        _visual(
            body,
            Cylinder(radius=0.005, length=0.006),
            Origin(xyz=(-0.3085, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            brass,
            name=f"side_hatch_bolt_{i}",
        )
    _visual(body, Box((0.245, 0.006, 0.130)), Origin(xyz=(-0.075, 0.223, 0.260)), brushed, name="rear_service_hatch")

    # Interior rails and heater elements are bracketed into the side walls.
    for z, label in ((0.170, "lower"), (0.330, "upper")):
        _x_cylinder(body, f"{label}_heater", (-0.075, -0.010, z), 0.009, 0.385, ceramic)
        _visual(body, Box((0.035, 0.030, 0.026)), Origin(xyz=(-0.270, -0.010, z)), brushed, name=f"{label}_heater_mount_0")
        _visual(body, Box((0.035, 0.030, 0.026)), Origin(xyz=(0.120, -0.010, z)), brushed, name=f"{label}_heater_mount_1")
    for z, label in ((0.210, "rack"), (0.305, "tray")):
        _visual(body, Box((0.050, 0.310, 0.010)), Origin(xyz=(-0.255, 0.035, z)), brushed, name=f"{label}_rail_0")
        _visual(body, Box((0.030, 0.310, 0.010)), Origin(xyz=(0.105, 0.035, z)), brushed, name=f"{label}_rail_1")
        for j, y in enumerate((-0.070, -0.010, 0.050, 0.110)):
            _x_cylinder(body, f"{label}_wire_{j}", (-0.075, y, z + 0.006), 0.0028, 0.360, brushed)

    # Exposed alternating bottom hinge knuckles on the fixed oven sill.
    _x_cylinder(body, "body_hinge_knuckle_0", (HINGE_X - 0.142, HINGE_Y, HINGE_Z), 0.013, 0.070, brushed)
    _x_cylinder(body, "body_hinge_knuckle_1", (HINGE_X, HINGE_Y, HINGE_Z), 0.013, 0.078, brushed)
    _x_cylinder(body, "body_hinge_knuckle_2", (HINGE_X + 0.142, HINGE_Y, HINGE_Z), 0.013, 0.070, brushed)
    _visual(body, Box((0.460, 0.018, 0.020)), Origin(xyz=(HINGE_X, HINGE_Y + 0.025, HINGE_Z + 0.035)), brushed, name="hinge_backer_bar")
    _visual(body, Box((0.055, 0.036, 0.055)), Origin(xyz=(HINGE_X - 0.250, HINGE_Y + 0.010, HINGE_Z + 0.018)), brushed, name="hinge_end_bracket_0")
    _visual(body, Box((0.055, 0.036, 0.055)), Origin(xyz=(HINGE_X + 0.250, HINGE_Y + 0.010, HINGE_Z + 0.018)), brushed, name="hinge_end_bracket_1")

    # Three bolted shaft adapters and bushings on the control face.
    knob_specs = (
        ("timer", 0.335, "line"),
        ("temperature", 0.250, "wedge"),
        ("mode", 0.165, "dot"),
    )
    for prefix, z, _indicator in knob_specs:
        _visual(body, Box((0.094, 0.006, 0.060)), Origin(xyz=(0.215, -0.248, z)), brushed, name=f"{prefix}_adapter")
        _y_cylinder(body, f"{prefix}_bushing", (0.215, -0.256, z), 0.017, 0.014, brass)
        for i, (dx, dz) in enumerate(((-0.037, -0.022), (0.037, -0.022), (-0.037, 0.022), (0.037, 0.022))):
            _front_disk(body, f"{prefix}_adapter_bolt_{i}", (0.215 + dx, -0.252, z + dz), 0.0045, 0.0060, brass)
    _visual(body, Box((0.050, 0.006, 0.018)), Origin(xyz=(0.215, -0.251, 0.386)), red, name="pilot_lens")

    # Four stubby feet under the metal shell.
    for i, x in enumerate((-0.230, 0.230)):
        for j, y in enumerate((-0.150, 0.150)):
            _visual(body, Cylinder(radius=0.025, length=0.040), Origin(xyz=(x, y, 0.065)), black, name=f"rubber_foot_{i}_{j}")

    # Bottom-hinged door.  The child frame lies on the hinge axis; in the
    # closed pose the door extends upward in local +Z, and positive rotation
    # around +X drops the free edge outward and down.
    door = model.part("door")
    _visual(door, Box((0.420, 0.022, 0.036)), Origin(xyz=(0.0, -0.008, 0.042)), brushed, name="lower_rail")
    _visual(door, Box((0.420, 0.022, 0.036)), Origin(xyz=(0.0, -0.008, 0.236)), brushed, name="top_rail")
    _visual(door, Box((0.036, 0.022, 0.230)), Origin(xyz=(-0.192, -0.008, 0.151)), brushed, name="side_rail_0")
    _visual(door, Box((0.036, 0.022, 0.230)), Origin(xyz=(0.192, -0.008, 0.151)), brushed, name="side_rail_1")
    _visual(door, Box((0.318, 0.008, 0.145)), Origin(xyz=(0.0, -0.020, 0.134)), glass, name="glass_pane")
    _visual(door, Box((0.320, 0.012, 0.018)), Origin(xyz=(0.0, -0.012, 0.210)), black, name="inner_gasket")
    _x_cylinder(door, "handle_grip", (0.0, -0.058, 0.220), 0.014, 0.315, black)
    _visual(door, Box((0.018, 0.048, 0.020)), Origin(xyz=(-0.135, -0.035, 0.220)), brushed, name="handle_standoff_0")
    _visual(door, Box((0.018, 0.048, 0.020)), Origin(xyz=(0.135, -0.035, 0.220)), brushed, name="handle_standoff_1")
    _x_cylinder(door, "door_hinge_knuckle_0", (-0.071, 0.0, 0.0), 0.012, 0.064, brushed)
    _x_cylinder(door, "door_hinge_knuckle_1", (0.071, 0.0, 0.0), 0.012, 0.064, brushed)
    _visual(door, Box((0.340, 0.012, 0.020)), Origin(xyz=(0.0, -0.023, 0.018)), brushed, name="door_hinge_leaf")
    _visual(door, Box((0.050, 0.026, 0.036)), Origin(xyz=(-0.071, -0.014, 0.018)), brushed, name="hinge_leaf_strap_0")
    _visual(door, Box((0.050, 0.026, 0.036)), Origin(xyz=(0.071, -0.014, 0.018)), brushed, name="hinge_leaf_strap_1")

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.42),
    )

    # Rotary knobs: each child has a visible metal shaft inserted into a brass
    # bushing plus a fluted bakelite cap with an indicator mark.
    knob_meshes = {}
    for prefix, _z, indicator in knob_specs:
        knob_geom = KnobGeometry(
            0.058,
            0.028,
            body_style="skirted",
            top_diameter=0.044,
            skirt=KnobSkirt(0.070, 0.006, flare=0.08, chamfer=0.0012),
            grip=KnobGrip(style="fluted", count=20, depth=0.0013),
            indicator=KnobIndicator(style=indicator, mode="raised", angle_deg=35.0),
            bore=KnobBore(style="d_shaft", diameter=0.008, flat_depth=0.001),
            center=False,
        )
        knob_meshes[prefix] = mesh_from_geometry(knob_geom, f"{prefix}_knob_cap")

    knob_limits = {
        "timer": MotionLimits(effort=1.2, velocity=4.0, lower=0.0, upper=5.15),
        "temperature": MotionLimits(effort=1.0, velocity=3.0, lower=-2.35, upper=2.35),
        "mode": MotionLimits(effort=0.9, velocity=2.5, lower=-1.05, upper=1.05),
    }
    for prefix, z, _indicator in knob_specs:
        knob = model.part(f"{prefix}_knob")
        knob.visual(
            Cylinder(radius=0.009, length=0.030),
            origin=Origin(xyz=(0.0, -0.010, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=brushed,
            name="shaft",
        )
        knob.visual(
            knob_meshes[prefix],
            origin=Origin(xyz=(0.0, -0.025, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
            material=black,
            name="cap",
        )
        model.articulation(
            f"{prefix}_shaft",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(0.215, -0.253, z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=knob_limits[prefix],
        )

    # Keep a direct reference alive for linters and for narrative clarity.
    _ = door_hinge
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("oven_body")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("door_hinge")

    # The metal shafts are deliberately captured by solid bushing proxies; this
    # is the small, hidden, mechanically correct overlap that makes the
    # shaft-driven controls read as manufacturable.
    for prefix in ("timer", "temperature", "mode"):
        knob = object_model.get_part(f"{prefix}_knob")
        ctx.allow_overlap(
            body,
            knob,
            elem_a=f"{prefix}_bushing",
            elem_b="shaft",
            reason="The control shaft is intentionally seated through the brass bushing proxy.",
        )
        ctx.expect_within(
            knob,
            body,
            axes="xz",
            inner_elem="shaft",
            outer_elem=f"{prefix}_bushing",
            margin=0.002,
            name=f"{prefix} shaft centered in bushing",
        )
        ctx.expect_overlap(
            knob,
            body,
            axes="y",
            elem_a="shaft",
            elem_b=f"{prefix}_bushing",
            min_overlap=0.008,
            name=f"{prefix} shaft retained in bushing",
        )

    with ctx.pose({door_hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="y",
            positive_elem="front_brow",
            negative_elem="top_rail",
            min_gap=0.003,
            max_gap=0.020,
            name="closed door sits just proud of front brow",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a="glass_pane",
            elem_b="oven_cavity",
            min_overlap=0.12,
            name="glass window covers the oven opening",
        )

    closed_top = ctx.part_element_world_aabb(door, elem="top_rail")
    with ctx.pose({door_hinge: 1.15}):
        open_top = ctx.part_element_world_aabb(door, elem="top_rail")
    ctx.check(
        "door opens downward and outward",
        closed_top is not None
        and open_top is not None
        and open_top[1][2] < closed_top[1][2] - 0.045
        and open_top[0][1] < closed_top[0][1] - 0.045,
        details=f"closed_top={closed_top}, open_top={open_top}",
    )

    for joint_name in ("timer_shaft", "temperature_shaft", "mode_shaft"):
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} is bounded revolute control",
            joint.articulation_type == ArticulationType.REVOLUTE
            and joint.motion_limits is not None
            and joint.motion_limits.lower is not None
            and joint.motion_limits.upper is not None
            and joint.motion_limits.upper > joint.motion_limits.lower,
            details=f"joint={joint}",
        )

    return ctx.report()


object_model = build_object_model()
