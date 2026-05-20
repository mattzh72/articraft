from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireSidewall,
    TireTread,
    TorusGeometry,
    WheelGeometry,
    mesh_from_geometry,
)


WIDTH = 0.55
DEPTH = 0.38
HEIGHT = 0.82
BOTTOM_Z = 0.13
TOP_Z = BOTTOM_Z + HEIGHT
WALL = 0.035
FAN_Z = BOTTOM_Z + 0.50 * HEIGHT
FRONT_Y = -DEPTH / 2.0
REAR_Y = DEPTH / 2.0

DOOR_WIDTH = 0.218
DOOR_PANEL_WIDTH = 0.200
DOOR_HEIGHT = 0.320
DOOR_Z = BOTTOM_Z + 0.60 * HEIGHT
DOOR_HINGE_X = -DOOR_WIDTH / 2.0
DOOR_HINGE_Y = REAR_Y + 0.010


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_evaporative_cooler")

    warm_plastic = model.material("warm_plastic", color=(0.78, 0.80, 0.76, 1.0))
    panel_plastic = model.material("panel_plastic", color=(0.17, 0.19, 0.20, 1.0))
    dark_plastic = model.material("dark_plastic", color=(0.03, 0.035, 0.04, 1.0))
    blue_tint = model.material("blue_tint", color=(0.18, 0.58, 0.90, 0.55))
    rubber = model.material("rubber", color=(0.015, 0.014, 0.013, 1.0))
    metal = model.material("brushed_metal", color=(0.62, 0.63, 0.60, 1.0))
    blade_blue = model.material("translucent_blade", color=(0.36, 0.63, 0.82, 0.82))

    housing = model.part("housing")

    # Boxy molded cooler body: side walls, back, bottom, top, and a front frame
    # around the fan opening.  The overlapping wall pieces intentionally form
    # one supported plastic shell rather than separate floating panels.
    housing.visual(
        Box((WALL, DEPTH, HEIGHT)),
        origin=Origin(xyz=(-WIDTH / 2.0 + WALL / 2.0, 0.0, BOTTOM_Z + HEIGHT / 2.0)),
        material=warm_plastic,
        name="side_wall_0",
    )
    housing.visual(
        Box((WALL, DEPTH, HEIGHT)),
        origin=Origin(xyz=(WIDTH / 2.0 - WALL / 2.0, 0.0, BOTTOM_Z + HEIGHT / 2.0)),
        material=warm_plastic,
        name="side_wall_1",
    )
    housing.visual(
        Box((WIDTH, DEPTH, WALL)),
        origin=Origin(xyz=(0.0, 0.0, TOP_Z - WALL / 2.0)),
        material=warm_plastic,
        name="top_panel",
    )
    housing.visual(
        Box((WIDTH, DEPTH, WALL)),
        origin=Origin(xyz=(0.0, 0.0, BOTTOM_Z + WALL / 2.0)),
        material=warm_plastic,
        name="bottom_panel",
    )
    housing.visual(
        Box((WIDTH, WALL, HEIGHT)),
        origin=Origin(xyz=(0.0, REAR_Y - WALL / 2.0, BOTTOM_Z + HEIGHT / 2.0)),
        material=warm_plastic,
        name="rear_panel",
    )

    opening_half = 0.195
    front_wall_center_y = FRONT_Y + WALL / 2.0
    side_strip_width = (WIDTH - 2.0 * opening_half) / 2.0
    housing.visual(
        Box((side_strip_width, WALL, HEIGHT)),
        origin=Origin(xyz=(-WIDTH / 2.0 + side_strip_width / 2.0, front_wall_center_y, BOTTOM_Z + HEIGHT / 2.0)),
        material=warm_plastic,
        name="front_side_0",
    )
    housing.visual(
        Box((side_strip_width, WALL, HEIGHT)),
        origin=Origin(xyz=(WIDTH / 2.0 - side_strip_width / 2.0, front_wall_center_y, BOTTOM_Z + HEIGHT / 2.0)),
        material=warm_plastic,
        name="front_side_1",
    )
    top_strip_height = TOP_Z - (FAN_Z + opening_half)
    bottom_strip_height = (FAN_Z - opening_half) - BOTTOM_Z
    housing.visual(
        Box((WIDTH, WALL, top_strip_height)),
        origin=Origin(xyz=(0.0, front_wall_center_y, TOP_Z - top_strip_height / 2.0)),
        material=warm_plastic,
        name="front_top",
    )
    housing.visual(
        Box((WIDTH, WALL, bottom_strip_height)),
        origin=Origin(xyz=(0.0, front_wall_center_y, BOTTOM_Z + bottom_strip_height / 2.0)),
        material=warm_plastic,
        name="front_bottom",
    )

    # Dark appliance control panel, water-level lens, and grille details.
    housing.visual(
        Box((0.245, 0.008, 0.115)),
        origin=Origin(xyz=(0.125, FRONT_Y - 0.002, TOP_Z - 0.095)),
        material=panel_plastic,
        name="control_panel",
    )
    housing.visual(
        Box((0.050, 0.007, 0.180)),
        origin=Origin(xyz=(-0.205, FRONT_Y - 0.002, BOTTOM_Z + 0.215)),
        material=blue_tint,
        name="water_window",
    )
    for i, x in enumerate((-0.060, -0.030, 0.030, 0.060)):
        housing.visual(
            Box((0.006, 0.006, 0.030)),
            origin=Origin(xyz=(0.125 + x, FRONT_Y - 0.007, TOP_Z - 0.045)),
            material=warm_plastic,
            name=f"flow_tick_{i}",
        )

    grille_y = FRONT_Y - 0.004
    for radius, tube, mesh_name in (
        (0.205, 0.006, "outer_grille_ring"),
        (0.135, 0.0045, "middle_grille_ring"),
        (0.070, 0.004, "inner_grille_ring"),
    ):
        housing.visual(
            mesh_from_geometry(TorusGeometry(radius, tube), mesh_name),
            origin=Origin(xyz=(0.0, grille_y, FAN_Z), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_plastic,
            name=mesh_name,
        )
    for i, angle in enumerate((0.0, pi / 4.0, -pi / 4.0, pi / 2.0)):
        housing.visual(
            Box((0.395, 0.008, 0.010)),
            origin=Origin(xyz=(0.0, grille_y, FAN_Z), rpy=(0.0, angle, 0.0)),
            material=dark_plastic,
            name=f"grille_spoke_{i}",
        )

    # Rear hinge-side hardware for the water fill panel door.
    hinge_radius = 0.010
    for i, (z_offset, length) in enumerate(((-0.145, 0.050), (0.0, 0.080), (0.145, 0.050))):
        housing.visual(
            Cylinder(radius=hinge_radius, length=length),
            origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_Z + z_offset)),
            material=metal,
            name=f"fixed_hinge_barrel_{i}",
        )
        housing.visual(
            Box((0.024, 0.006, length)),
            origin=Origin(xyz=(DOOR_HINGE_X - 0.014, REAR_Y + 0.004, DOOR_Z + z_offset)),
            material=metal,
            name=f"fixed_hinge_leaf_{i}",
        )

    # Four supported caster assemblies mounted to the bottom panel.
    caster_origins = [
        (-0.205, -0.125),
        (0.205, -0.125),
        (-0.205, 0.125),
        (0.205, 0.125),
    ]
    for i, (x, y) in enumerate(caster_origins):
        caster = model.part(f"caster_{i}")
        caster.visual(
            Cylinder(radius=0.008, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, -0.025)),
            material=metal,
            name="caster_stem",
        )
        caster.visual(
            Cylinder(radius=0.020, length=0.010),
            origin=Origin(xyz=(0.0, 0.0, -0.052)),
            material=metal,
            name="swivel_plate",
        )
        caster.visual(
            Box((0.006, 0.020, 0.060)),
            origin=Origin(xyz=(-0.024, 0.0, -0.080)),
            material=metal,
            name="fork_side_0",
        )
        caster.visual(
            Box((0.006, 0.020, 0.060)),
            origin=Origin(xyz=(0.024, 0.0, -0.080)),
            material=metal,
            name="fork_side_1",
        )
        caster.visual(
            Cylinder(radius=0.0045, length=0.060),
            origin=Origin(xyz=(0.0, 0.0, -0.088), rpy=(0.0, pi / 2.0, 0.0)),
            material=metal,
            name="axle",
        )
        caster.visual(
            mesh_from_geometry(
                TireGeometry(
                    0.034,
                    0.028,
                    inner_radius=0.023,
                    tread=TireTread(style="ribbed", depth=0.0016, count=14),
                    sidewall=TireSidewall(style="rounded", bulge=0.03),
                ),
                f"caster_tire_{i}",
            ),
            origin=Origin(xyz=(0.0, 0.0, -0.088)),
            material=rubber,
            name="caster_tire",
        )
        caster.visual(
            mesh_from_geometry(WheelGeometry(0.023, 0.026), f"caster_wheel_{i}"),
            origin=Origin(xyz=(0.0, 0.0, -0.088)),
            material=metal,
            name="caster_wheel",
        )
        model.articulation(
            f"housing_to_caster_{i}",
            ArticulationType.FIXED,
            parent=housing,
            child=caster,
            origin=Origin(xyz=(x, y, BOTTOM_Z)),
        )

    # Continuously rotating axial fan blade set, inside the front grille.
    fan_blade = model.part("fan_blade")
    fan_blade.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                0.160,
                0.042,
                5,
                thickness=0.028,
                blade_pitch_deg=32.0,
                blade_sweep_deg=28.0,
                blade=FanRotorBlade(shape="scimitar", tip_pitch_deg=14.0, camber=0.18),
                hub=FanRotorHub(style="spinner", bore_diameter=0.010),
            ),
            "fan_rotor",
        ),
        material=blade_blue,
        name="fan_rotor",
    )
    model.articulation(
        "housing_to_fan_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=fan_blade,
        origin=Origin(xyz=(0.0, FRONT_Y + 0.018, FAN_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=80.0),
    )

    # Rear water fill door, framed by a side vertical hinge.
    fill_door = model.part("fill_door")
    fill_door.visual(
        Box((DOOR_PANEL_WIDTH, 0.014, DOOR_HEIGHT)),
        origin=Origin(xyz=(0.018 + DOOR_PANEL_WIDTH / 2.0, -0.002, 0.0)),
        material=warm_plastic,
        name="door_panel",
    )
    fill_door.visual(
        Box((DOOR_PANEL_WIDTH - 0.040, 0.006, DOOR_HEIGHT - 0.060)),
        origin=Origin(xyz=(0.018 + DOOR_PANEL_WIDTH / 2.0, 0.006, 0.0)),
        material=model.material("slightly_darker_plastic", color=(0.66, 0.69, 0.66, 1.0)),
        name="raised_door_center",
    )
    fill_door.visual(
        Box((0.048, 0.012, 0.110)),
        origin=Origin(xyz=(DOOR_WIDTH - 0.038, 0.013, -0.010)),
        material=panel_plastic,
        name="recessed_pull",
    )
    for i, z_offset in enumerate((-0.090, 0.090)):
        fill_door.visual(
            Cylinder(radius=hinge_radius, length=0.060),
            origin=Origin(xyz=(0.0, 0.0, z_offset)),
            material=metal,
            name=f"door_hinge_barrel_{i}",
        )
        fill_door.visual(
            Box((0.037, 0.008, 0.060)),
            origin=Origin(xyz=(0.0265, -0.002, z_offset)),
            material=metal,
            name=f"door_hinge_leaf_{i}",
        )
    model.articulation(
        "housing_to_fill_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=fill_door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    # Continuous flow-rate control knob on the front control panel.
    flow_knob = model.part("flow_knob")
    flow_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.060,
                0.030,
                body_style="skirted",
                top_diameter=0.044,
                edge_radius=0.001,
                grip=KnobGrip(style="fluted", count=20, depth=0.0015),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
                center=False,
            ),
            "flow_knob_cap",
        ),
        material=warm_plastic,
        name="knob_cap",
    )
    model.articulation(
        "housing_to_flow_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=flow_knob,
        origin=Origin(xyz=(0.125, FRONT_Y - 0.006, TOP_Z - 0.090), rpy=(pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("fill_door")
    fan = object_model.get_part("fan_blade")
    knob = object_model.get_part("flow_knob")
    door_joint = object_model.get_articulation("housing_to_fill_door")
    fan_joint = object_model.get_articulation("housing_to_fan_blade")
    knob_joint = object_model.get_articulation("housing_to_flow_knob")

    ctx.check("rear fill door is revolute", door_joint.articulation_type == ArticulationType.REVOLUTE)
    ctx.check("fan blade spins continuously", fan_joint.articulation_type == ArticulationType.CONTINUOUS)
    ctx.check("flow knob rotates continuously", knob_joint.articulation_type == ArticulationType.CONTINUOUS)

    ctx.expect_gap(
        door,
        housing,
        axis="y",
        positive_elem="door_panel",
        negative_elem="rear_panel",
        min_gap=0.0,
        max_gap=0.004,
        name="fill door sits just proud of rear panel",
    )
    ctx.expect_overlap(
        door,
        housing,
        axes="xz",
        elem_a="door_panel",
        elem_b="rear_panel",
        min_overlap=0.18,
        name="fill door is mounted on rear face",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_joint: 1.40}):
        open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "fill door opens outward from rear",
        closed_aabb is not None
        and open_aabb is not None
        and float(open_aabb[1][1]) > float(closed_aabb[1][1]) + 0.12,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    for part_obj, name in ((fan, "fan blade present"), (knob, "flow knob present")):
        ctx.check(name, part_obj is not None)

    return ctx.report()


object_model = build_object_model()
