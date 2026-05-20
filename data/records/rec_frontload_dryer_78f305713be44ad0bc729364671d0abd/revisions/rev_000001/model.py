from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 72) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _annulus_mesh(
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    name: str,
    *,
    segments: int = 96,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _circle_profile(outer_radius, segments),
            [_circle_profile(inner_radius, segments)],
            thickness,
            center=True,
            cap=True,
            closed=True,
        ),
        name,
    )


def _open_drum_mesh() -> MeshGeometry:
    """A connected open-front dryer drum with a rear plate and rolled front lip.

    The mesh is authored around the local X axle so the drum can spin directly
    about its part frame's +X axis.
    """

    seg = 96
    radius = 0.48
    length = 0.78
    front_x = -length / 2.0
    rear_x = length / 2.0
    lip_depth = 0.065
    inner_lip_radius = 0.37
    geom = MeshGeometry()

    front_outer: list[int] = []
    rear_outer: list[int] = []
    front_inner: list[int] = []
    lip_inner: list[int] = []

    for i in range(seg):
        a = 2.0 * math.pi * i / seg
        y = radius * math.cos(a)
        z = radius * math.sin(a)
        front_outer.append(geom.add_vertex(front_x, y, z))
        rear_outer.append(geom.add_vertex(rear_x, y, z))

        yi = inner_lip_radius * math.cos(a)
        zi = inner_lip_radius * math.sin(a)
        front_inner.append(geom.add_vertex(front_x, yi, zi))
        lip_inner.append(geom.add_vertex(front_x + lip_depth, yi, zi))

    rear_center = geom.add_vertex(rear_x, 0.0, 0.0)

    for i in range(seg):
        j = (i + 1) % seg

        # Outer cylindrical drum wall.
        geom.add_face(front_outer[i], rear_outer[i], rear_outer[j])
        geom.add_face(front_outer[i], rear_outer[j], front_outer[j])

        # Rear cap.
        geom.add_face(rear_center, rear_outer[j], rear_outer[i])

        # Flat annular front lip, leaving a large open mouth.
        geom.add_face(front_outer[i], front_outer[j], front_inner[j])
        geom.add_face(front_outer[i], front_inner[j], front_inner[i])

        # Short return surface inside the rolled front lip.
        geom.add_face(front_inner[i], front_inner[j], lip_inner[j])
        geom.add_face(front_inner[i], lip_inner[j], lip_inner[i])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_industrial_dryer")

    stainless = model.material("brushed_stainless", rgba=(0.72, 0.74, 0.73, 1.0))
    dark_stainless = model.material("dark_stainless", rgba=(0.34, 0.36, 0.36, 1.0))
    black = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    drum_metal = model.material("perforated_drum_steel", rgba=(0.80, 0.82, 0.80, 1.0))
    hinge_metal = model.material("hinge_pin_steel", rgba=(0.55, 0.56, 0.56, 1.0))
    glass = model.material("blue_smoked_glass", rgba=(0.35, 0.62, 0.82, 0.42))
    panel_black = model.material("black_control_panel", rgba=(0.04, 0.045, 0.05, 1.0))
    button_red = model.material("red_start_button", rgba=(0.82, 0.05, 0.04, 1.0))

    body = model.part("body")

    # Wide stainless cabinet shell.  The front is made from individual panels so
    # the porthole is truly open rather than hidden in a solid block.
    body.visual(Box((1.10, 0.040, 1.54)), origin=Origin(xyz=(0.0, -0.795, 0.84)), material=stainless, name="side_panel_0")
    body.visual(Box((1.10, 0.040, 1.54)), origin=Origin(xyz=(0.0, 0.795, 0.84)), material=stainless, name="side_panel_1")
    body.visual(Box((1.10, 1.63, 0.045)), origin=Origin(xyz=(0.0, 0.0, 1.625)), material=stainless, name="top_panel")
    body.visual(Box((1.10, 1.63, 0.055)), origin=Origin(xyz=(0.0, 0.0, 0.055)), material=stainless, name="bottom_panel")
    body.visual(Box((0.040, 1.63, 1.54)), origin=Origin(xyz=(0.550, 0.0, 0.84)), material=stainless, name="rear_panel")

    # Front pieces around the large drum opening.
    body.visual(Box((0.040, 1.63, 0.330)), origin=Origin(xyz=(-0.565, 0.0, 1.445)), material=stainless, name="front_upper_panel")
    body.visual(Box((0.040, 1.63, 0.340)), origin=Origin(xyz=(-0.565, 0.0, 0.255)), material=stainless, name="front_lower_panel")
    body.visual(Box((0.040, 0.350, 0.930)), origin=Origin(xyz=(-0.565, -0.635, 0.850)), material=stainless, name="front_jamb_0")
    body.visual(Box((0.040, 0.350, 0.930)), origin=Origin(xyz=(-0.565, 0.635, 0.850)), material=stainless, name="front_jamb_1")

    body.visual(
        _annulus_mesh(0.465, 0.375, 0.040, "body_port_ring"),
        origin=Origin(xyz=(-0.600, 0.0, 0.860), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="port_ring",
    )

    # Service plinth, adjustable feet, lower vent slots, and rear bearing boss.
    body.visual(Box((1.05, 1.45, 0.100)), origin=Origin(xyz=(0.0, 0.0, 0.090)), material=dark_stainless, name="base_plinth")
    for i, y in enumerate((-0.62, 0.62)):
        for j, x in enumerate((-0.36, 0.36)):
            body.visual(
                Cylinder(radius=0.055, length=0.075),
                origin=Origin(xyz=(x, y, 0.015)),
                material=dark_stainless,
                name=f"leveling_foot_{i}_{j}",
            )
    for i, z in enumerate((0.235, 0.285, 0.335)):
        body.visual(
            Box((0.012, 0.56, 0.018)),
            origin=Origin(xyz=(-0.588, 0.0, z)),
            material=panel_black,
            name=f"vent_slot_{i}",
        )
    body.visual(
        Cylinder(radius=0.082, length=0.070),
        origin=Origin(xyz=(0.515, 0.0, 0.860), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_stainless,
        name="rear_bearing_boss",
    )

    # Raised commercial control box on the front upper panel.
    body.visual(
        Box((0.060, 0.600, 0.210)),
        origin=Origin(xyz=(-0.596, 0.420, 1.435)),
        material=panel_black,
        name="control_panel",
    )
    body.visual(Box((0.010, 0.190, 0.055)), origin=Origin(xyz=(-0.626, 0.250, 1.475)), material=glass, name="display_window")

    # Two exposed barrel hinge assemblies: cabinet leaf, pin barrel, and screw heads.
    for name, z in (("lower", 0.620), ("upper", 1.100)):
        body.visual(
            Box((0.080, 0.150, 0.185)),
            origin=Origin(xyz=(-0.615, -0.482, z)),
            material=hinge_metal,
            name=f"{name}_hinge_leaf",
        )
        body.visual(
            Cylinder(radius=0.022, length=0.230),
            origin=Origin(xyz=(-0.625, -0.450, z)),
            material=hinge_metal,
            name=f"{name}_barrel",
        )
        for dz in (-0.055, 0.055):
            body.visual(
                Cylinder(radius=0.014, length=0.006),
                origin=Origin(xyz=(-0.653, -0.505, z + dz), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=dark_stainless,
                name=f"{name}_hinge_screw_{dz:+.3f}",
            )

    drum = model.part("drum")
    drum.visual(mesh_from_geometry(_open_drum_mesh(), "open_cylindrical_drum"), material=drum_metal, name="drum_shell")
    drum.visual(
        Cylinder(radius=0.032, length=0.960),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="axle_shaft",
    )
    # Three lifter ribs make the drum's rotation visible and read as a real dryer drum.
    for i, theta in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        radial = 0.445
        drum.visual(
            Box((0.600, 0.055, 0.080)),
            origin=Origin(
                xyz=(0.0, radial * math.sin(theta), radial * math.cos(theta)),
                rpy=(-theta, 0.0, 0.0),
            ),
            material=drum_metal,
            name=f"lifter_rib_{i}",
        )

    model.articulation(
        "drum_axle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=drum,
        origin=Origin(xyz=(0.0, 0.0, 0.860)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=8.0),
    )

    door = model.part("door")
    door.visual(
        _annulus_mesh(0.450, 0.315, 0.055, "round_porthole_door"),
        # Part frame is the left hinge line.  At q=0 the round door extends to
        # local +Y, so positive rotation about +Z swings it outward toward -X.
        origin=Origin(xyz=(-0.040, 0.450, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="door_ring",
    )
    door.visual(
        Cylinder(radius=0.330, length=0.020),
        origin=Origin(xyz=(-0.043, 0.450, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="glass_window",
    )
    door.visual(
        _annulus_mesh(0.343, 0.315, 0.024, "black_inner_gasket"),
        origin=Origin(xyz=(-0.072, 0.450, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="inner_gasket",
    )
    for name, z in (("lower", -0.240), ("upper", 0.240)):
        door.visual(
            Box((0.060, 0.130, 0.130)),
            origin=Origin(xyz=(-0.040, 0.055, z)),
            material=hinge_metal,
            name=f"{name}_door_hinge_leaf",
        )
    door.visual(
        Box((0.066, 0.140, 0.320)),
        origin=Origin(xyz=(-0.040, 0.925, 0.0)),
        material=stainless,
        name="latch_tab",
    )

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.645, -0.450, 0.860)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=2.05),
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=0.036, length=0.018),
        origin=Origin(xyz=(-0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="pivot_pad",
    )
    latch.visual(
        Box((0.018, 0.024, 0.300)),
        origin=Origin(xyz=(-0.009, 0.0, 0.0)),
        material=hinge_metal,
        name="latch_backplate",
    )
    for name, z in (("lower", -0.125), ("upper", 0.125)):
        latch.visual(
            Cylinder(radius=0.010, length=0.080),
            origin=Origin(xyz=(-0.049, 0.0, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_metal,
            name=f"{name}_standoff",
        )
    latch.visual(
        Cylinder(radius=0.016, length=0.360),
        origin=Origin(xyz=(-0.092, 0.0, 0.0)),
        material=dark_stainless,
        name="pull_bar",
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(-0.073, 0.945, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=0.45),
    )

    cycle_knob = model.part("cycle_knob")
    cycle_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.080,
                0.038,
                body_style="skirted",
                top_diameter=0.060,
                grip=KnobGrip(style="fluted", count=20, depth=0.0014),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
                center=False,
            ),
            "cycle_knob",
        ),
        origin=Origin(rpy=(0.0, -math.pi / 2.0, 0.0)),
        material=dark_stainless,
        name="knob_cap",
    )
    model.articulation(
        "cycle_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cycle_knob,
        origin=Origin(xyz=(-0.630, 0.385, 1.430)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0),
    )

    start_button = model.part("start_button")
    start_button.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(-0.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=button_red,
        name="button_cap",
    )
    model.articulation(
        "start_button_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=start_button,
        origin=Origin(xyz=(-0.626, 0.545, 1.430)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=0.20, lower=0.0, upper=0.018),
    )

    # Keep a handle to silence static analysis complaints in environments that
    # report unused variables for authored joints while preserving semantic name.
    _ = door_hinge
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch")
    drum_axle = object_model.get_articulation("drum_axle")
    door_hinge = object_model.get_articulation("door_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")

    ctx.check(
        "drum spins on a front-back axle",
        tuple(round(v, 3) for v in drum_axle.axis) == (1.0, 0.0, 0.0),
        details=f"axis={drum_axle.axis}",
    )
    ctx.expect_overlap(
        door,
        body,
        axes="yz",
        elem_a="door_ring",
        elem_b="port_ring",
        min_overlap=0.70,
        name="porthole door aligns over cabinet opening",
    )
    ctx.expect_within(
        door,
        body,
        axes="yz",
        inner_elem="glass_window",
        outer_elem="port_ring",
        margin=0.05,
        name="wide glass remains inside the porthole surround",
    )

    closed_ring = ctx.part_element_world_aabb(door, elem="door_ring")
    closed_lifter = ctx.part_element_world_aabb(drum, elem="lifter_rib_0")
    closed_latch = ctx.part_element_world_aabb(latch, elem="pull_bar")

    with ctx.pose({door_hinge: 1.35}):
        open_ring = ctx.part_element_world_aabb(door, elem="door_ring")
    with ctx.pose({drum_axle: 1.10}):
        spun_lifter = ctx.part_element_world_aabb(drum, elem="lifter_rib_0")
    with ctx.pose({latch_pivot: 0.35}):
        swung_latch = ctx.part_element_world_aabb(latch, elem="pull_bar")

    def _center(aabb, idx: int) -> float | None:
        if aabb is None:
            return None
        return 0.5 * (aabb[0][idx] + aabb[1][idx])

    ctx.check(
        "left-edge door hinge swings the porthole outward",
        closed_ring is not None
        and open_ring is not None
        and _center(open_ring, 0) is not None
        and _center(closed_ring, 0) is not None
        and _center(open_ring, 0) < _center(closed_ring, 0) - 0.18,
        details=f"closed={closed_ring}, open={open_ring}",
    )
    ctx.check(
        "drum lifter visibly changes position when axle rotates",
        closed_lifter is not None
        and spun_lifter is not None
        and abs((_center(spun_lifter, 1) or 0.0) - (_center(closed_lifter, 1) or 0.0)) > 0.20,
        details=f"closed={closed_lifter}, spun={spun_lifter}",
    )
    ctx.check(
        "bar-pull latch pivots separately from the door",
        closed_latch is not None
        and swung_latch is not None
        and abs((_center(swung_latch, 1) or 0.0) - (_center(closed_latch, 1) or 0.0)) > 0.025,
        details=f"closed={closed_latch}, swung={swung_latch}",
    )

    return ctx.report()


object_model = build_object_model()
