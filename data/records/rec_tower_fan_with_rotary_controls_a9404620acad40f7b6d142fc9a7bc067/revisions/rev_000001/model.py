from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    Inertial,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
)


JOINT_Z = 0.060
TRAY_Z = 0.422
BLOWER_CENTER = (0.0, -0.004, 0.225)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_office_tower_fan")

    warm_white = model.material("warm_white_plastic", rgba=(0.88, 0.88, 0.84, 1.0))
    light_gray = model.material("light_gray_plastic", rgba=(0.62, 0.64, 0.63, 1.0))
    dark_gray = model.material("charcoal_plastic", rgba=(0.08, 0.09, 0.10, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.025, 0.025, 0.025, 1.0))
    satin_metal = model.material("satin_blower_metal", rgba=(0.66, 0.69, 0.70, 1.0))
    accent = model.material("cool_blue_indicator", rgba=(0.10, 0.38, 0.80, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.096, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=light_gray,
        name="base_disk",
    )
    base.visual(
        Cylinder(radius=0.077, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=warm_white,
        name="base_dish",
    )
    base.visual(
        Cylinder(radius=0.049, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=warm_white,
        name="bearing_pedestal",
    )
    base.visual(
        Cylinder(radius=0.041, length=0.017),
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z - 0.0085)),
        material=dark_gray,
        name="bearing_socket",
    )
    base.visual(
        Cylinder(radius=0.099, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=rubber,
        name="rubber_foot",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.096, length=0.060),
        mass=0.82,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    tower = model.part("tower")
    # Readable rotating collar and neck above the stationary bearing.
    tower.visual(
        Cylinder(radius=0.032, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=light_gray,
        name="pivot_collar",
    )
    tower.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.031)),
        material=warm_white,
        name="pivot_neck",
    )

    # Short hollow rectangular tower shell, with rounded corner posts and a
    # real front grille rather than a flat painted rectangle.
    tower.visual(
        Box((0.140, 0.010, 0.360)),
        origin=Origin(xyz=(0.0, 0.045, 0.232)),
        material=warm_white,
        name="rear_shell",
    )
    for index, x in enumerate((-0.066, 0.066)):
        tower.visual(
            Box((0.014, 0.096, 0.360)),
            origin=Origin(xyz=(x, 0.0, 0.232)),
            material=warm_white,
            name=f"side_shell_{index}",
        )
    for index, (x, y) in enumerate(
        ((-0.061, -0.043), (0.061, -0.043), (-0.061, 0.043), (0.061, 0.043))
    ):
        tower.visual(
            Cylinder(radius=0.009, length=0.360),
            origin=Origin(xyz=(x, y, 0.232)),
            material=warm_white,
            name=f"corner_post_{index}",
        )
    tower.visual(
        Box((0.140, 0.100, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.0525)),
        material=warm_white,
        name="lower_cap",
    )
    tower.visual(
        Box((0.140, 0.100, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.405)),
        material=warm_white,
        name="top_cap",
    )
    tower.visual(
        mesh_from_geometry(
            VentGrilleGeometry(
                (0.126, 0.292),
                frame=0.010,
                face_thickness=0.004,
                duct_depth=0.006,
                slat_pitch=0.017,
                slat_width=0.008,
                slat_angle_deg=18.0,
                corner_radius=0.008,
                slats=VentGrilleSlats(
                    profile="airfoil",
                    direction="down",
                    inset=0.001,
                    divider_count=2,
                    divider_width=0.0035,
                ),
                frame_profile=VentGrilleFrame(style="radiused", depth=0.001),
                sleeve=VentGrilleSleeve(style="none"),
                center=False,
            ),
            "tower_front_grille",
        ),
        origin=Origin(xyz=(0.0, -0.050, 0.220), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="front_grille",
    )
    tower.visual(
        Box((0.038, 0.003, 0.012)),
        origin=Origin(xyz=(0.0, -0.051, 0.386)),
        material=accent,
        name="front_badge",
    )

    # Recessed top control deck: darker tray floor below a raised perimeter rim.
    tower.visual(
        Box((0.112, 0.064, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, TRAY_Z - 0.002)),
        material=dark_gray,
        name="control_tray",
    )
    tower.visual(
        Box((0.124, 0.007, 0.012)),
        origin=Origin(xyz=(0.0, -0.0355, TRAY_Z + 0.002)),
        material=warm_white,
        name="front_tray_lip",
    )
    tower.visual(
        Box((0.124, 0.007, 0.012)),
        origin=Origin(xyz=(0.0, 0.0355, TRAY_Z + 0.002)),
        material=warm_white,
        name="rear_tray_lip",
    )
    for index, x in enumerate((-0.0615, 0.0615)):
        tower.visual(
            Box((0.007, 0.064, 0.012)),
            origin=Origin(xyz=(x, 0.0, TRAY_Z + 0.002)),
            material=warm_white,
            name=f"side_tray_lip_{index}",
        )
    tower.visual(
        mesh_from_geometry(TorusGeometry(radius=0.014, tube=0.002, radial_segments=18), "button_guide_ring"),
        origin=Origin(xyz=(0.041, 0.015, TRAY_Z + 0.0015)),
        material=light_gray,
        name="button_guide",
    )
    tower.inertial = Inertial.from_geometry(
        Box((0.150, 0.110, 0.430)),
        mass=0.92,
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
    )

    model.articulation(
        "base_to_tower",
        ArticulationType.REVOLUTE,
        parent=base,
        child=tower,
        origin=Origin(xyz=(0.0, 0.0, JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.6, velocity=0.85, lower=-1.05, upper=1.05),
    )

    blower_wheel = model.part("blower_wheel")
    blower_wheel.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                0.034,
                0.015,
                0.270,
                26,
                blade_thickness=0.0017,
                blade_sweep_deg=28.0,
                backplate=True,
                shroud=True,
            ),
            "vertical_blower_wheel",
        ),
        material=satin_metal,
        name="blower_cage",
    )
    blower_wheel.visual(
        Cylinder(radius=0.0055, length=0.325),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=dark_gray,
        name="blower_shaft",
    )
    blower_wheel.visual(
        Cylinder(radius=0.016, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_gray,
        name="blower_hub",
    )
    blower_wheel.visual(
        Box((0.004, 0.012, 0.055)),
        origin=Origin(xyz=(0.029, -0.002, 0.010)),
        material=accent,
        name="spin_marker",
    )
    blower_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.034, length=0.270),
        mass=0.18,
    )
    model.articulation(
        "tower_to_blower_wheel",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=blower_wheel,
        origin=Origin(xyz=BLOWER_CENTER),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.16, velocity=32.0),
    )

    knob_meshes = [
        mesh_from_geometry(
            KnobGeometry(
                0.026,
                0.015,
                body_style="domed",
                edge_radius=0.0008,
                grip=KnobGrip(style="ribbed", count=18, depth=0.0006, width=0.0012),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "speed_dial_cap",
        ),
        mesh_from_geometry(
            KnobGeometry(
                0.024,
                0.014,
                body_style="faceted",
                base_diameter=0.026,
                top_diameter=0.020,
                edge_radius=0.0006,
                grip=KnobGrip(style="ribbed", count=14, depth=0.0005, width=0.0013),
                indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
                center=False,
            ),
            "timer_dial_cap",
        ),
    ]
    for index, x in enumerate((-0.030, 0.004)):
        dial = model.part(f"dial_{index}")
        dial.visual(
            knob_meshes[index],
            origin=Origin(),
            material=dark_gray,
            name="dial_cap",
        )
        dial.visual(
            Box((0.010, 0.0018, 0.0012)),
            origin=Origin(xyz=(0.004, 0.0, 0.0145)),
            material=accent,
            name="dial_marker",
        )
        dial.inertial = Inertial.from_geometry(
            Cylinder(radius=0.013, length=0.016),
            mass=0.018,
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
        )
        model.articulation(
            f"tower_to_dial_{index}",
            ArticulationType.CONTINUOUS,
            parent=tower,
            child=dial,
            origin=Origin(xyz=(x, -0.006, TRAY_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.05, velocity=8.0),
        )

    button = model.part("oscillation_button")
    button.visual(
        Cylinder(radius=0.0070, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_gray,
        name="button_stem",
    )
    button.visual(
        Cylinder(radius=0.0105, length=0.0045),
        origin=Origin(xyz=(0.0, 0.0, 0.01425)),
        material=accent,
        name="button_cap",
    )
    button.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0105, length=0.0165),
        mass=0.010,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )
    model.articulation(
        "tower_to_oscillation_button",
        ArticulationType.PRISMATIC,
        parent=tower,
        child=button,
        origin=Origin(xyz=(0.041, 0.015, TRAY_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=0.05, lower=0.0, upper=0.004),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    tower = object_model.get_part("tower")
    blower = object_model.get_part("blower_wheel")
    dial_0 = object_model.get_part("dial_0")
    button = object_model.get_part("oscillation_button")

    base_to_tower = object_model.get_articulation("base_to_tower")
    blower_joint = object_model.get_articulation("tower_to_blower_wheel")
    dial_joint = object_model.get_articulation("tower_to_dial_0")
    button_joint = object_model.get_articulation("tower_to_oscillation_button")

    ctx.check(
        "primary_joints_present",
        base_to_tower is not None
        and blower_joint.articulation_type == ArticulationType.CONTINUOUS
        and dial_joint.articulation_type == ArticulationType.CONTINUOUS
        and button_joint.articulation_type == ArticulationType.PRISMATIC,
        "Expected oscillation, blower, dial, and push-button mechanisms.",
    )

    ctx.expect_gap(
        tower,
        base,
        axis="z",
        positive_elem="pivot_collar",
        negative_elem="bearing_socket",
        max_gap=0.0008,
        max_penetration=0.0,
        name="oscillation collar sits on bearing",
    )
    ctx.expect_overlap(
        blower,
        tower,
        axes="xz",
        elem_a="blower_cage",
        elem_b="front_grille",
        min_overlap=0.050,
        name="blower wheel is aligned behind grille",
    )
    ctx.expect_gap(
        dial_0,
        tower,
        axis="z",
        positive_elem="dial_cap",
        negative_elem="control_tray",
        max_gap=0.001,
        max_penetration=0.0,
        name="dial sits on recessed tray",
    )
    ctx.expect_gap(
        button,
        tower,
        axis="z",
        positive_elem="button_stem",
        negative_elem="control_tray",
        max_gap=0.001,
        max_penetration=0.0,
        name="button plunger is supported by tray",
    )

    def _elem_center(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((float(mins[i]) + float(maxs[i])) * 0.5 for i in range(3))

    badge_rest = _elem_center(tower, "front_badge")
    with ctx.pose({base_to_tower: 0.75}):
        badge_swept = _elem_center(tower, "front_badge")
    ctx.check(
        "tower oscillates about vertical base joint",
        badge_rest is not None
        and badge_swept is not None
        and abs(badge_swept[0] - badge_rest[0]) > 0.015,
        details=f"rest={badge_rest}, swept={badge_swept}",
    )

    marker_rest = _elem_center(blower, "spin_marker")
    with ctx.pose({blower_joint: 1.1}):
        marker_spun = _elem_center(blower, "spin_marker")
    ctx.check(
        "blower wheel has visible continuous spin",
        marker_rest is not None
        and marker_spun is not None
        and abs(marker_spun[1] - marker_rest[1]) > 0.010,
        details=f"rest={marker_rest}, spun={marker_spun}",
    )

    dial_marker_rest = _elem_center(dial_0, "dial_marker")
    with ctx.pose({dial_joint: 1.2}):
        dial_marker_rotated = _elem_center(dial_0, "dial_marker")
    ctx.check(
        "rotary dial marker moves with control",
        dial_marker_rest is not None
        and dial_marker_rotated is not None
        and abs(dial_marker_rotated[1] - dial_marker_rest[1]) > 0.003,
        details=f"rest={dial_marker_rest}, rotated={dial_marker_rotated}",
    )

    button_rest = ctx.part_world_position(button)
    with ctx.pose({button_joint: 0.004}):
        button_pressed = ctx.part_world_position(button)
    ctx.check(
        "oscillation button depresses independently",
        button_rest is not None
        and button_pressed is not None
        and button_pressed[2] < button_rest[2] - 0.003,
        details=f"rest={button_rest}, pressed={button_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
