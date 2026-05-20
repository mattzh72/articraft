from __future__ import annotations

from math import cos, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


BODY_WIDTH = 0.130
BODY_DEPTH = 0.032
BODY_HEIGHT = 0.230
BODY_Z = BODY_HEIGHT * 0.5
REAR_Y = -BODY_DEPTH * 0.5
FRONT_Y = BODY_DEPTH * 0.5
KICKSTAND_THETA = 2.15
KICKSTAND_LENGTH = 0.105


def _mat(name: str, color) -> Material:
    return Material(name, rgba=color)


def _slot_panel(name: str, width: float, height: float):
    return mesh_from_geometry(
        SlotPatternPanelGeometry(
            (width, height),
            0.0020,
            slot_size=(0.014, 0.0028),
            pitch=(0.021, 0.011),
            frame=0.004,
            corner_radius=0.002,
            stagger=True,
        ),
        name,
    )


def _rounded_slab_mesh():
    profile = rounded_rect_profile(
        BODY_WIDTH,
        BODY_DEPTH,
        0.010,
        corner_segments=10,
    )
    return mesh_from_geometry(
        ExtrudeGeometry(profile, BODY_HEIGHT, center=True),
        "rounded_router_slab",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_router_kickstand")

    warm_white = model.material("warm_white", rgba=(0.86, 0.84, 0.78, 1.0))
    satin_black = model.material("satin_black", rgba=(0.025, 0.027, 0.030, 1.0))
    soft_black = model.material("soft_black", rgba=(0.055, 0.058, 0.062, 1.0))
    dark_recess = model.material("dark_recess", rgba=(0.010, 0.012, 0.014, 1.0))
    rubber = model.material("rubber", rgba=(0.020, 0.020, 0.020, 1.0))
    led_blue = model.material("led_blue", rgba=(0.05, 0.40, 1.0, 1.0))
    led_green = model.material("led_green", rgba=(0.05, 0.95, 0.34, 1.0))
    led_dim = model.material("led_dim", rgba=(0.65, 0.70, 0.67, 1.0))
    metal_pin = model.material("metal_pin", rgba=(0.55, 0.57, 0.58, 1.0))

    housing = model.part("housing")
    housing.visual(
        _rounded_slab_mesh(),
        origin=Origin(xyz=(0.0, 0.0, BODY_Z)),
        material=warm_white,
        name="rounded_slab",
    )

    # Front cosmetic panel, LED light pipe, and lower rubber edge.
    housing.visual(
        Box((0.088, 0.0022, 0.160)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.0002, 0.126)),
        material=satin_black,
        name="front_inset",
    )
    housing.visual(
        Box((0.0035, 0.0026, 0.128)),
        origin=Origin(xyz=(-0.026, FRONT_Y + 0.0010, 0.126)),
        material=warm_white,
        name="front_light_pipe",
    )
    for index, (z, material) in enumerate(
        ((0.172, led_blue), (0.145, led_green), (0.118, led_green), (0.091, led_dim))
    ):
        housing.visual(
            Box((0.0065, 0.0030, 0.0065)),
            origin=Origin(xyz=(-0.026, FRONT_Y + 0.0022, z)),
            material=material,
            name=f"status_led_{index}",
        )
    housing.visual(
        Box((0.120, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=rubber,
        name="lower_edge_foot",
    )

    # Rear face details: a dark kickstand pocket flanked by real slotted vents.
    rear_panel_rpy = (1.57079632679, 0.0, 0.0)
    for suffix, x in (("0", -0.041), ("1", 0.041)):
        housing.visual(
            _slot_panel(f"rear_vent_{suffix}", 0.030, 0.115),
            origin=Origin(xyz=(x, REAR_Y - 0.0008, 0.145), rpy=rear_panel_rpy),
            material=soft_black,
            name=f"rear_vent_{suffix}",
        )
    housing.visual(
        Box((0.036, 0.0035, 0.115)),
        origin=Origin(xyz=(0.0, REAR_Y - 0.0008, 0.104)),
        material=dark_recess,
        name="kickstand_slot",
    )
    housing.visual(
        Box((0.048, 0.0040, 0.011)),
        origin=Origin(xyz=(0.0, REAR_Y - 0.0012, 0.061)),
        material=soft_black,
        name="kickstand_hinge_pocket",
    )

    # Top antenna hinge pedestals: child antenna barrels sit on these saddles.
    for socket_name, pin_name, x in (
        ("antenna_socket_0", "antenna_pin_0", -0.052),
        ("antenna_socket_1", "antenna_pin_1", 0.052),
    ):
        housing.visual(
            Box((0.024, 0.013, 0.007)),
            origin=Origin(xyz=(x, REAR_Y - 0.0015, BODY_HEIGHT + 0.0035)),
            material=soft_black,
            name=socket_name,
        )
        housing.visual(
            Cylinder(radius=0.0055, length=0.021),
            origin=Origin(
                xyz=(x, REAR_Y - 0.0015, BODY_HEIGHT + 0.0075),
                rpy=(0.0, 1.57079632679, 0.0),
            ),
            material=metal_pin,
            name=pin_name,
        )

    # Lower rear kickstand hinge lugs.  The moving barrel fits between them.
    for suffix, x in (("0", -0.022), ("1", 0.022)):
        housing.visual(
            Box((0.012, 0.011, 0.017)),
            origin=Origin(xyz=(x, REAR_Y - 0.0055, 0.060)),
            material=soft_black,
            name=f"kickstand_lug_{suffix}",
        )
        housing.visual(
            Cylinder(radius=0.0052, length=0.012),
            origin=Origin(
                xyz=(x, REAR_Y - 0.0078, 0.060),
                rpy=(0.0, 1.57079632679, 0.0),
            ),
            material=metal_pin,
            name=f"kickstand_pin_{suffix}",
        )

    for suffix, x in (("0", -0.052), ("1", 0.052)):
        antenna = model.part(f"antenna_{suffix}")
        antenna.visual(
            Cylinder(radius=0.0052, length=0.018),
            origin=Origin(rpy=(0.0, 1.57079632679, 0.0)),
            material=soft_black,
            name="hinge_barrel",
        )
        antenna.visual(
            Cylinder(radius=0.0040, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, 0.012)),
            material=soft_black,
            name="root_neck",
        )
        antenna.visual(
            Cylinder(radius=0.0032, length=0.108),
            origin=Origin(xyz=(0.0, 0.0, 0.066)),
            material=satin_black,
            name="antenna_whip",
        )
        antenna.visual(
            Sphere(radius=0.0050),
            origin=Origin(xyz=(0.0, 0.0, 0.122)),
            material=satin_black,
            name="tip_cap",
        )
        model.articulation(
            f"housing_to_antenna_{suffix}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=antenna,
            origin=Origin(xyz=(x, REAR_Y - 0.0015, BODY_HEIGHT + 0.0128)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.2,
                velocity=2.5,
                lower=-0.55,
                upper=1.25,
            ),
        )

    kickstand = model.part("kickstand")
    kickstand.visual(
        Cylinder(radius=0.0054, length=0.025),
        origin=Origin(rpy=(0.0, 1.57079632679, 0.0)),
        material=soft_black,
        name="hinge_barrel",
    )
    leg_mid = (
        0.0,
        -sin(KICKSTAND_THETA) * KICKSTAND_LENGTH * 0.5,
        cos(KICKSTAND_THETA) * KICKSTAND_LENGTH * 0.5,
    )
    leg_end = (
        0.0,
        -sin(KICKSTAND_THETA) * KICKSTAND_LENGTH,
        cos(KICKSTAND_THETA) * KICKSTAND_LENGTH,
    )
    kickstand.visual(
        Box((0.018, 0.009, KICKSTAND_LENGTH)),
        origin=Origin(xyz=leg_mid, rpy=(KICKSTAND_THETA, 0.0, 0.0)),
        material=soft_black,
        name="support_leg",
    )
    kickstand.visual(
        Box((0.050, 0.018, 0.006)),
        origin=Origin(xyz=leg_end),
        material=rubber,
        name="foot_pad",
    )
    model.articulation(
        "housing_to_kickstand",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=kickstand,
        origin=Origin(xyz=(0.0, REAR_Y - 0.0078, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.0,
            lower=-KICKSTAND_THETA,
            upper=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    antenna_0 = object_model.get_part("antenna_0")
    antenna_1 = object_model.get_part("antenna_1")
    kickstand = object_model.get_part("kickstand")
    antenna_joint_0 = object_model.get_articulation("housing_to_antenna_0")
    kickstand_joint = object_model.get_articulation("housing_to_kickstand")

    def _center_of_aabb(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    housing_aabb = ctx.part_world_aabb(housing)
    ctx.check(
        "housing stands on lower edge",
        housing_aabb is not None
        and abs(housing_aabb[0][2]) < 0.002
        and housing_aabb[1][2] > 0.22,
        details=f"housing_aabb={housing_aabb}",
    )

    for antenna, pin_name, check_suffix in (
        (antenna_0, "antenna_pin_0", "0"),
        (antenna_1, "antenna_pin_1", "1"),
    ):
        ctx.allow_overlap(
            antenna,
            housing,
            elem_a="hinge_barrel",
            elem_b=pin_name,
            reason="A metal hinge pin is intentionally captured inside the antenna root barrel.",
        )
        ctx.expect_overlap(
            antenna,
            housing,
            axes="x",
            elem_a="hinge_barrel",
            elem_b=pin_name,
            min_overlap=0.016,
            name=f"antenna {check_suffix} pin remains captured in barrel",
        )

    ctx.expect_gap(
        antenna_0,
        housing,
        axis="z",
        min_gap=-0.0005,
        max_gap=0.0025,
        positive_elem="hinge_barrel",
        negative_elem="antenna_socket_0",
        name="antenna 0 barrel seats on top socket",
    )
    ctx.expect_gap(
        antenna_1,
        housing,
        axis="z",
        min_gap=-0.0005,
        max_gap=0.0025,
        positive_elem="hinge_barrel",
        negative_elem="antenna_socket_1",
        name="antenna 1 barrel seats on top socket",
    )
    ctx.expect_overlap(
        kickstand,
        housing,
        axes="x",
        elem_a="hinge_barrel",
        elem_b="kickstand_hinge_pocket",
        min_overlap=0.020,
        name="kickstand hinge barrel lies across rear pocket",
    )

    tip_at_rest = _center_of_aabb(ctx.part_element_world_aabb(antenna_0, elem="tip_cap"))
    with ctx.pose({antenna_joint_0: 0.80}):
        tip_tilted = _center_of_aabb(ctx.part_element_world_aabb(antenna_0, elem="tip_cap"))
    ctx.check(
        "antenna hinge tilts rearward",
        tip_at_rest is not None
        and tip_tilted is not None
        and tip_tilted[1] < tip_at_rest[1] - 0.065
        and tip_tilted[2] < tip_at_rest[2] - 0.025,
        details=f"rest_tip={tip_at_rest}, tilted_tip={tip_tilted}",
    )

    foot_open = _center_of_aabb(ctx.part_element_world_aabb(kickstand, elem="foot_pad"))
    with ctx.pose({kickstand_joint: -KICKSTAND_THETA}):
        foot_folded = _center_of_aabb(ctx.part_element_world_aabb(kickstand, elem="foot_pad"))
    ctx.check(
        "kickstand folds up into rear slot",
        foot_open is not None
        and foot_folded is not None
        and foot_folded[2] > foot_open[2] + 0.080
        and foot_folded[1] > foot_open[1] + 0.060,
        details=f"open_foot={foot_open}, folded_foot={foot_folded}",
    )

    return ctx.report()


object_model = build_object_model()
