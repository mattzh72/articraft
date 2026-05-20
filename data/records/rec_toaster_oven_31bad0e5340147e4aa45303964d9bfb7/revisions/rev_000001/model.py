from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _cq_box(size: tuple[float, float, float], center: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center)


def _build_body_shell():
    """One low-part-count bent sheet-metal oven shell with stamped front features."""
    depth = 0.360
    width = 0.460
    height = 0.280
    sheet = 0.008
    front_x = -depth / 2.0

    # Bent outer wrap: top/bottom/sides/back plus a front frame and integral
    # right-side control panel.  Small overlaps make the authored sheet read as
    # one manufacturable welded/hemmed stamping instead of loose panels.
    shell = _cq_box((depth, width, sheet), (0.0, 0.0, height - sheet / 2.0))
    for part in (
        _cq_box((depth, width, sheet), (0.0, 0.0, sheet / 2.0)),
        _cq_box((depth, sheet, height), (0.0, -width / 2.0 + sheet / 2.0, height / 2.0)),
        _cq_box((depth, sheet, height), (0.0, width / 2.0 - sheet / 2.0, height / 2.0)),
        _cq_box((sheet, width, height), (depth / 2.0 - sheet / 2.0, 0.0, height / 2.0)),
        # front top and bottom rails around the door opening
        _cq_box((sheet, width, 0.040), (front_x + sheet / 2.0, 0.0, 0.260)),
        _cq_box((sheet, width, 0.044), (front_x + sheet / 2.0, 0.0, 0.022)),
        # left jamb and center divider, leaving a single broad door aperture
        _cq_box((sheet, 0.022, 0.198), (front_x + sheet / 2.0, -0.219, 0.139)),
        _cq_box((sheet, 0.022, 0.198), (front_x + sheet / 2.0, 0.105, 0.139)),
        # flat integral control panel rather than a separate cosmetic insert
        _cq_box((sheet, 0.108, 0.198), (front_x + sheet / 2.0, 0.171, 0.139)),
        # cheap folded lips that stiffen the large front opening
        _cq_box((0.024, 0.010, 0.166), (front_x + 0.010, -0.204, 0.140)),
        _cq_box((0.024, 0.010, 0.166), (front_x + 0.010, 0.090, 0.140)),
    ):
        shell = shell.union(part)

    # Three punched shaft holes in the control panel, sized with real clearance
    # for low-cost plastic knobs on steel D-shafts.
    for z in (0.198, 0.140, 0.082):
        cutter = (
            cq.Workplane("YZ")
            .center(0.171, z)
            .circle(0.0085)
            .extrude(0.060, both=True)
            .translate((front_x + sheet / 2.0, 0.0, 0.0))
        )
        shell = shell.cut(cutter)

    # Punched cooling slots: rectangular cuts are cheaper than separate vents.
    for y in (-0.105, -0.070, -0.035, 0.000, 0.035):
        shell = shell.cut(_cq_box((0.145, 0.012, 0.030), (0.030, y, height + 0.001)))
    for z in (0.154, 0.177, 0.200, 0.223):
        shell = shell.cut(_cq_box((0.125, 0.030, 0.010), (0.040, -width / 2.0, z)))
        shell = shell.cut(_cq_box((0.125, 0.030, 0.010), (0.040, width / 2.0, z)))

    return shell


def _build_door_frame():
    """One-piece molded/stamped door frame with captured glass window."""
    width = 0.325
    height = 0.205
    thick_x = 0.018
    bar = 0.020
    # The door panel sits slightly behind the hinge pin so the barrel clears the
    # folded front lip while the glass/frame face still lands close to the shell.
    x_center = -0.003
    frame = _cq_box((thick_x, width, bar), (x_center, 0.0, bar / 2.0))
    for part in (
        _cq_box((thick_x, width, bar), (x_center, 0.0, height - bar / 2.0)),
        _cq_box((thick_x, bar, height - 0.020), (x_center, -width / 2.0 + bar / 2.0, height / 2.0)),
        _cq_box((thick_x, bar, height - 0.020), (x_center, width / 2.0 - bar / 2.0, height / 2.0)),
        # four small molded snap/heat-stake pads retain the glass pane
        _cq_box((0.006, 0.030, 0.010), (x_center - 0.011, -0.105, 0.038)),
        _cq_box((0.006, 0.030, 0.010), (x_center - 0.011, 0.105, 0.038)),
        _cq_box((0.006, 0.030, 0.010), (x_center - 0.011, -0.105, 0.172)),
        _cq_box((0.006, 0.030, 0.010), (x_center - 0.011, 0.105, 0.172)),
    ):
        frame = frame.union(part)
    return frame


def _cq_cylinder_y(radius: float, length: float, y_center: float, x: float, z: float):
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, y_center, 0.0))
    )


def _build_hinge_barrels():
    barrel = _cq_cylinder_y(0.006, 0.070, -0.110, 0.0, 0.0)
    barrel = barrel.union(_cq_cylinder_y(0.006, 0.070, 0.110, 0.0, 0.0))
    return barrel


def _build_hinge_pin():
    return _cq_cylinder_y(0.0032, 0.350, -0.055, -0.190, 0.048)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_toaster_oven")

    stainless = model.material("brushed_stainless", rgba=(0.70, 0.70, 0.66, 1.0))
    dark_metal = model.material("black_powdercoat", rgba=(0.015, 0.016, 0.017, 1.0))
    black_plastic = model.material("molded_black_plastic", rgba=(0.025, 0.025, 0.026, 1.0))
    glass = model.material("smoked_glass", rgba=(0.20, 0.29, 0.35, 0.45))
    steel = model.material("zinc_plated_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    ceramic = model.material("white_ceramic", rgba=(0.86, 0.82, 0.72, 1.0))
    heater = model.material("warm_heating_element", rgba=(1.0, 0.33, 0.06, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_build_body_shell(), "body_shell", tolerance=0.0015),
        material=stainless,
        name="shell",
    )

    # A dark rear cavity panel makes the oven read hollow through the glass door.
    body.visual(
        Box((0.006, 0.285, 0.165)),
        origin=Origin(xyz=(0.170, -0.055, 0.140)),
        material=dark_metal,
        name="rear_liner",
    )

    # Two cheap resistance loops with ceramic end blocks.  The blocks bridge the
    # elements into the stamped side wall/divider so the static internals have a
    # believable support path.
    for i, (x, z) in enumerate(((-0.045, 0.074), (0.085, 0.074), (-0.045, 0.207), (0.085, 0.207))):
        body.visual(
            Cylinder(radius=0.0032, length=0.292),
            origin=Origin(xyz=(x, -0.055, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=heater,
            name=f"heater_rod_{i}",
        )
        body.visual(
            Box((0.024, 0.030, 0.018)),
            origin=Origin(xyz=(x, -0.208, z)),
            material=ceramic,
            name=f"heater_socket_{i}_0",
        )
        body.visual(
            Box((0.024, 0.030, 0.018)),
            origin=Origin(xyz=(x, 0.098, z)),
            material=ceramic,
            name=f"heater_socket_{i}_1",
        )

    # One continuous pin retained by two end tabs: low part count and obvious
    # assembly order (pin through door barrels, then clip/peen at the tabs).
    body.visual(
        mesh_from_cadquery(_build_hinge_pin(), "hinge_pin", tolerance=0.0008),
        material=steel,
        name="hinge_pin",
    )
    for j, y in enumerate((-0.230, 0.120)):
        body.visual(
            Box((0.020, 0.012, 0.036)),
            origin=Origin(xyz=(-0.181, y, 0.050)),
            material=stainless,
            name=f"hinge_tab_{j}",
        )

    # Front-panel screw heads show how the shell/control face is assembled
    # without adding separate cosmetic fascias.
    for k, (y, z) in enumerate(((0.128, 0.232), (0.214, 0.232), (0.128, 0.048), (0.214, 0.048))):
        body.visual(
            Cylinder(radius=0.004, length=0.002),
            origin=Origin(xyz=(-0.181, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name=f"screw_head_{k}",
        )

    door = model.part("door")
    door.visual(
        mesh_from_cadquery(_build_door_frame(), "door_frame", tolerance=0.001),
        material=dark_metal,
        name="door_frame",
    )
    door.visual(
        Box((0.004, 0.286, 0.154)),
        origin=Origin(xyz=(-0.012, 0.0, 0.105)),
        material=glass,
        name="glass_pane",
    )
    door.visual(
        Cylinder(radius=0.007, length=0.250),
        origin=Origin(xyz=(-0.036, 0.0, 0.165), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_plastic,
        name="handle_bar",
    )
    for j, y in enumerate((-0.095, 0.095)):
        door.visual(
            Cylinder(radius=0.0045, length=0.048),
            origin=Origin(xyz=(-0.017, y, 0.165), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_plastic,
            name=f"handle_post_{j}",
        )
    door.visual(
        mesh_from_cadquery(_build_hinge_barrels(), "door_hinge_barrels", tolerance=0.0008),
        material=steel,
        name="hinge_barrels",
    )

    door_hinge = model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.190, -0.055, 0.048)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=2.0, lower=0.0, upper=1.35),
    )

    knob_meshes = []
    for name, diameter in (("mode", 0.050), ("temp", 0.048), ("timer", 0.048)):
        knob_geom = KnobGeometry(
            diameter,
            0.028,
            body_style="skirted",
            top_diameter=diameter * 0.72,
            skirt=KnobSkirt(diameter + 0.010, 0.0055, flare=0.08, chamfer=0.001),
            grip=KnobGrip(style="fluted", count=18, depth=0.0013),
            indicator=KnobIndicator(style="line", mode="raised", depth=0.0008, angle_deg=90.0),
            bore=KnobBore(style="d_shaft", diameter=0.006, flat_depth=0.001),
        )
        knob_meshes.append((name, mesh_from_geometry(knob_geom, f"{name}_knob_cap")))

    knob_specs = (
        ("mode_knob", knob_meshes[0][1], 0.198, 5.24),
        ("temp_knob", knob_meshes[1][1], 0.140, 4.60),
        ("timer_knob", knob_meshes[2][1], 0.082, 5.50),
    )
    for name, mesh, z, upper in knob_specs:
        knob = model.part(name)
        knob.visual(
            mesh,
            origin=Origin(xyz=(-0.014, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_plastic,
            name="cap",
        )
        knob.visual(
            Cylinder(radius=0.011, length=0.003),
            origin=Origin(xyz=(-0.0015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black_plastic,
            name="panel_collar",
        )
        knob.visual(
            Cylinder(radius=0.005, length=0.040),
            origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel,
            name="shaft",
        )
        model.articulation(
            f"body_to_{name}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=knob,
            origin=Origin(xyz=(-0.180, 0.171, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=0.6, velocity=6.0, lower=0.0, upper=upper),
        )

    # Keep a reference alive for tests/readability; the object model owns it.
    _ = door_hinge
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    door = object_model.get_part("door")
    hinge = object_model.get_articulation("body_to_door")

    ctx.allow_overlap(
        body,
        door,
        elem_a="hinge_pin",
        elem_b="hinge_barrels",
        reason="The steel hinge pin is intentionally captured inside the door barrel proxy.",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            body,
            door,
            axis="x",
            positive_elem="shell",
            negative_elem="door_frame",
            min_gap=0.0005,
            max_gap=0.006,
            name="closed door sits just in front of shell",
        )
        ctx.expect_within(
            body,
            door,
            axes="xz",
            inner_elem="hinge_pin",
            outer_elem="hinge_barrels",
            margin=0.002,
            name="hinge pin is coaxial with door barrels",
        )
        ctx.expect_overlap(
            body,
            door,
            axes="y",
            elem_a="hinge_pin",
            elem_b="hinge_barrels",
            min_overlap=0.05,
            name="hinge pin has retained barrel engagement",
        )
        closed_aabb = ctx.part_element_world_aabb(door, elem="door_frame")

    with ctx.pose({hinge: 1.35}):
        open_aabb = ctx.part_element_world_aabb(door, elem="door_frame")

    ctx.check(
        "door swings downward and outward",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] < closed_aabb[1][2] - 0.040
        and open_aabb[0][0] < closed_aabb[0][0] - 0.120,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    for knob_name in ("mode_knob", "temp_knob", "timer_knob"):
        knob = object_model.get_part(knob_name)
        ctx.expect_overlap(
            body,
            knob,
            axes="x",
            elem_a="shell",
            elem_b="shaft",
            min_overlap=0.010,
            name=f"{knob_name} shaft enters punched control-panel hole",
        )

    return ctx.report()


object_model = build_object_model()
