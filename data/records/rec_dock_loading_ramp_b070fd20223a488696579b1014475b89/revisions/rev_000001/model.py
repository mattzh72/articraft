from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireShoulder,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yard_ramp")

    steel = model.material("weathered_steel", rgba=(0.32, 0.34, 0.34, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    safety_yellow = model.material("worn_yellow", rgba=(0.95, 0.68, 0.12, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    wheel_metal = model.material("wheel_metal", rgba=(0.55, 0.56, 0.54, 1.0))

    deck_length = 5.50
    deck_width = 1.80
    deck_thickness = 0.10
    deck_angle = math.radians(10.0)
    deck_center_z = 1.00
    slope_rpy = (0.0, -deck_angle, 0.0)
    cyl_y_on_slope_rpy = (-math.pi / 2.0, -deck_angle, 0.0)

    cos_t = math.cos(deck_angle)
    sin_t = math.sin(deck_angle)

    def deck_point(x: float, y: float, z: float) -> tuple[float, float, float]:
        """Map local deck-plane coordinates into the root/world frame."""
        return (
            cos_t * x - sin_t * z,
            y,
            deck_center_z + sin_t * x + cos_t * z,
        )

    deck = model.part("deck")
    deck.visual(
        Box((deck_length, deck_width, deck_thickness)),
        origin=Origin(xyz=(0.0, 0.0, deck_center_z), rpy=slope_rpy),
        material=steel,
        name="deck_plate",
    )

    # Raised side curbs and transverse cleats give the long steel deck the
    # recognizable yard-ramp grating/traction silhouette.
    curb_width = 0.08
    curb_height = 0.22
    for side, y in enumerate((-deck_width / 2.0 - curb_width / 2.0, deck_width / 2.0 + curb_width / 2.0)):
        deck.visual(
            Box((deck_length - 0.18, curb_width, curb_height)),
            origin=Origin(
                xyz=deck_point(0.0, y, deck_thickness / 2.0 + curb_height / 2.0),
                rpy=slope_rpy,
            ),
            material=dark_steel,
            name=f"side_curb_{side}",
        )

    for i, x in enumerate((-2.05, -1.45, -0.85, -0.25, 0.35, 0.95, 1.55, 2.15)):
        deck.visual(
            Box((0.045, deck_width - 0.18, 0.020)),
            origin=Origin(
                xyz=deck_point(x, 0.0, deck_thickness / 2.0 + 0.010),
                rpy=slope_rpy,
            ),
            material=dark_steel,
            name=f"traction_cleat_{i}",
        )

    for i, y in enumerate((-0.62, 0.0, 0.62)):
        deck.visual(
            Box((deck_length - 0.35, 0.08, 0.18)),
            origin=Origin(
                xyz=deck_point(-0.05, y, -deck_thickness / 2.0 - 0.09),
                rpy=slope_rpy,
            ),
            material=dark_steel,
            name=f"underside_stringer_{i}",
        )

    hinge_local_x = deck_length / 2.0 + 0.055
    hinge_local_z = deck_thickness / 2.0 + 0.045
    hinge_xyz = deck_point(hinge_local_x, 0.0, hinge_local_z)
    deck.visual(
        Cylinder(radius=0.022, length=deck_width + 0.24),
        origin=Origin(xyz=hinge_xyz, rpy=cyl_y_on_slope_rpy),
        material=dark_steel,
        name="front_hinge_pin",
    )
    for i, y in enumerate((-deck_width / 2.0 - 0.04, deck_width / 2.0 + 0.04)):
        deck.visual(
            Box((0.13, 0.10, 0.10)),
            origin=Origin(
                xyz=deck_point(hinge_local_x - 0.018, y, deck_thickness / 2.0 + 0.040),
                rpy=slope_rpy,
            ),
            material=dark_steel,
            name=f"hinge_end_support_{i}",
        )

    axle_x = deck_point(-2.15, 0.0, 0.0)[0]
    axle_z = 0.34
    deck.visual(
        Cylinder(radius=0.045, length=2.55),
        origin=Origin(xyz=(axle_x, 0.0, axle_z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_axle",
    )

    underside_z = deck_point(-2.15, 0.0, -deck_thickness / 2.0)[2]
    hanger_height = underside_z - axle_z + 0.10
    for i, y in enumerate((-0.72, 0.72)):
        deck.visual(
            Box((0.16, 0.10, hanger_height)),
            origin=Origin(xyz=(axle_x, y, axle_z + hanger_height / 2.0 - 0.035)),
            material=dark_steel,
            name=f"axle_hanger_{i}",
        )
    for i, y in enumerate((-0.97, 0.97)):
        deck.visual(
            Box((0.18, 0.08, 0.36)),
            origin=Origin(xyz=(axle_x, y, axle_z + 0.13)),
            material=dark_steel,
            name=f"outer_fork_{i}",
        )

    lip_plate = model.part("lip_plate")
    lip_plate.visual(
        Box((0.78, deck_width - 0.14, 0.070)),
        origin=Origin(xyz=(0.43, 0.0, -0.018)),
        material=safety_yellow,
        name="lip_panel",
    )
    lip_plate.visual(
        Cylinder(radius=0.045, length=deck_width - 0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="lip_hinge_barrel",
    )
    lip_plate.visual(
        Box((0.08, deck_width - 0.22, 0.080)),
        origin=Origin(xyz=(0.085, 0.0, -0.030)),
        material=safety_yellow,
        name="lip_root_weld",
    )

    model.articulation(
        "deck_to_lip",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=lip_plate,
        origin=Origin(xyz=hinge_xyz, rpy=slope_rpy),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=160.0, velocity=0.6, lower=-0.25, upper=0.95),
    )

    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.32,
            0.20,
            inner_radius=0.235,
            tread=TireTread(style="block", depth=0.018, count=22, land_ratio=0.55),
            grooves=(TireGroove(center_offset=0.0, width=0.020, depth=0.006),),
            sidewall=TireSidewall(style="square", bulge=0.03),
            shoulder=TireShoulder(width=0.018, radius=0.006),
        ),
        "rear_tire",
    )
    rim_mesh = mesh_from_geometry(
        WheelGeometry(
            0.235,
            0.18,
            rim=WheelRim(inner_radius=0.135, flange_height=0.018, flange_thickness=0.008),
            hub=WheelHub(
                radius=0.075,
                width=0.15,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=6, circle_diameter=0.095, hole_diameter=0.010),
            ),
            face=WheelFace(dish_depth=0.010, front_inset=0.004, rear_inset=0.004),
            spokes=WheelSpokes(style="straight", count=6, thickness=0.010, window_radius=0.035),
            bore=WheelBore(style="round", diameter=0.090),
        ),
        "rear_wheel_rim",
    )

    wheel_centers = ((axle_x, -1.15, axle_z), (axle_x, 1.15, axle_z))
    for i, xyz in enumerate(wheel_centers):
        wheel = model.part(f"wheel_{i}")
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=black_rubber,
            name="tire",
        )
        wheel.visual(
            rim_mesh,
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=wheel_metal,
            name="rim",
        )
        model.articulation(
            f"deck_to_wheel_{i}",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=wheel,
            origin=Origin(xyz=xyz),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=90.0, velocity=8.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    lip = object_model.get_part("lip_plate")
    lip_joint = object_model.get_articulation("deck_to_lip")
    wheel_0 = object_model.get_part("wheel_0")
    wheel_1 = object_model.get_part("wheel_1")

    ctx.allow_overlap(
        deck,
        lip,
        elem_a="front_hinge_pin",
        elem_b="lip_hinge_barrel",
        reason="The lip barrel is intentionally captured around the steel hinge pin.",
    )
    ctx.allow_overlap(
        deck,
        wheel_0,
        elem_a="rear_axle",
        elem_b="rim",
        reason="The wheel rim is modeled as a bearing sleeve rotating around the rear axle.",
    )
    ctx.allow_overlap(
        deck,
        wheel_1,
        elem_a="rear_axle",
        elem_b="rim",
        reason="The wheel rim is modeled as a bearing sleeve rotating around the rear axle.",
    )

    ctx.expect_overlap(
        deck,
        lip,
        axes="y",
        elem_a="front_hinge_pin",
        elem_b="lip_hinge_barrel",
        min_overlap=1.55,
        name="lip hinge spans the deck width",
    )
    ctx.expect_overlap(
        wheel_0,
        deck,
        axes="y",
        elem_a="rim",
        elem_b="rear_axle",
        min_overlap=0.08,
        name="wheel 0 centered on rear axle",
    )
    ctx.expect_overlap(
        wheel_1,
        deck,
        axes="y",
        elem_a="rim",
        elem_b="rear_axle",
        min_overlap=0.08,
        name="wheel 1 centered on rear axle",
    )

    rest_aabb = ctx.part_world_aabb(lip)
    with ctx.pose({lip_joint: 0.75}):
        lowered_aabb = ctx.part_world_aabb(lip)
    ctx.check(
        "lip plate hinges downward",
        rest_aabb is not None
        and lowered_aabb is not None
        and lowered_aabb[0][2] < rest_aabb[0][2] - 0.10,
        details=f"rest={rest_aabb}, lowered={lowered_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
