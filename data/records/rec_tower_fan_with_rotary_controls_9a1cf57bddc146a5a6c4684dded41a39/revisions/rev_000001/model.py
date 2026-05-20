from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BlowerWheelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    VentGrilleFrame,
    VentGrilleGeometry,
    VentGrilleSlats,
    VentGrilleSleeve,
    mesh_from_geometry,
    superellipse_profile,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="slim_household_tower_fan")

    warm_white = model.material("warm_white", rgba=(0.86, 0.86, 0.80, 1.0))
    pearl_white = model.material("pearl_white", rgba=(0.96, 0.95, 0.90, 1.0))
    satin_black = model.material("satin_black", rgba=(0.025, 0.027, 0.030, 1.0))
    dark_graphite = model.material("dark_graphite", rgba=(0.10, 0.11, 0.12, 1.0))
    soft_gray = model.material("soft_gray", rgba=(0.52, 0.54, 0.55, 1.0))
    rubber = model.material("rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    # Home appliance scale: roughly one meter tall, with a slim oval footprint.
    body_width = 0.170
    body_depth = 0.125
    body_height = 0.940
    neck_height = 0.032
    body_top_z = neck_height + body_height

    base = model.part("base")
    base_profile = superellipse_profile(0.360, 0.255, exponent=2.7, segments=72)
    base.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(base_profile, 0.055, cap=True, closed=True),
            "rounded_base",
        ),
        material=warm_white,
        name="rounded_base",
    )
    base.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                superellipse_profile(0.315, 0.210, exponent=2.7, segments=72),
                0.006,
                cap=True,
                closed=True,
            ),
            "rubber_foot",
        ),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material=rubber,
        name="rubber_foot",
    )
    base.visual(
        Cylinder(radius=0.070, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=dark_graphite,
        name="oscillation_bearing",
    )

    body = model.part("body")
    body.visual(
        Cylinder(radius=0.058, length=neck_height),
        origin=Origin(xyz=(0.0, 0.0, neck_height / 2.0)),
        material=pearl_white,
        name="turntable_neck",
    )
    body.visual(
        Cylinder(radius=0.068, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=dark_graphite,
        name="turntable_shadow",
    )

    # The body is built as a real open frame rather than a solid block: side
    # stiles, top and bottom caps, a shallow rear shell, and a full-height grille.
    body.visual(
        Box((0.030, body_depth - 0.026, body_height)),
        origin=Origin(xyz=(-body_width / 2.0 + 0.019, 0.0, neck_height + body_height / 2.0)),
        material=pearl_white,
        name="side_stile_0",
    )
    body.visual(
        Box((0.030, body_depth - 0.026, body_height)),
        origin=Origin(xyz=(body_width / 2.0 - 0.019, 0.0, neck_height + body_height / 2.0)),
        material=pearl_white,
        name="side_stile_1",
    )
    for x in (-body_width / 2.0 + 0.018, body_width / 2.0 - 0.018):
        for y in (-body_depth / 2.0 + 0.018, body_depth / 2.0 - 0.018):
            body.visual(
                Cylinder(radius=0.018, length=body_height),
                origin=Origin(xyz=(x, y, neck_height + body_height / 2.0)),
                material=pearl_white,
                name=f"rounded_corner_{0 if x < 0 else 1}_{0 if y < 0 else 1}",
            )
    body.visual(
        mesh_from_geometry(
            ExtrudeGeometry.from_z0(
                superellipse_profile(body_width, body_depth, exponent=2.7, segments=64),
                0.080,
                cap=True,
                closed=True,
            ),
            "lower_cap",
        ),
        origin=Origin(xyz=(0.0, 0.0, neck_height)),
        material=pearl_white,
        name="lower_cap",
    )
    button_hole = [
        (x, y - 0.030)
        for x, y in superellipse_profile(0.038, 0.032, exponent=2.4, segments=32)
    ]
    body.visual(
        mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                superellipse_profile(body_width, body_depth, exponent=2.7, segments=64),
                (button_hole,),
                0.080,
                cap=True,
                center=True,
                closed=True,
            ),
            "top_cap",
        ),
        origin=Origin(xyz=(0.0, 0.0, body_top_z - 0.040)),
        material=pearl_white,
        name="top_cap",
    )
    body.visual(
        Box((0.116, 0.006, 0.840)),
        origin=Origin(xyz=(0.0, body_depth / 2.0 - 0.004, neck_height + 0.480)),
        material=warm_white,
        name="rear_shell_panel",
    )
    body.visual(
        mesh_from_geometry(
            VentGrilleGeometry(
                (0.124, 0.805),
                frame=0.008,
                face_thickness=0.003,
                duct_depth=0.014,
                duct_wall=0.002,
                slat_pitch=0.014,
                slat_width=0.006,
                slat_angle_deg=18.0,
                corner_radius=0.012,
                slats=VentGrilleSlats(
                    profile="boxed",
                    direction="down",
                    inset=0.0015,
                    divider_count=1,
                    divider_width=0.004,
                ),
                frame_profile=VentGrilleFrame(style="radiused", depth=0.0012),
                sleeve=VentGrilleSleeve(style="short", depth=0.010, wall=0.002),
            ),
            "front_grille",
        ),
        origin=Origin(
            xyz=(0.0, -body_depth / 2.0 - 0.006, neck_height + 0.445),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=satin_black,
        name="front_grille",
    )
    body.visual(
        Box((0.006, 0.010, 0.760)),
        origin=Origin(xyz=(0.0, -body_depth / 2.0 - 0.011, neck_height + 0.445)),
        material=soft_gray,
        name="grille_center_spine",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, neck_height + 0.814)),
        material=dark_graphite,
        name="upper_shaft_bearing",
    )
    body.visual(
        Box((0.006, 0.020, 0.050)),
        origin=Origin(xyz=(0.010, 0.0, neck_height + 0.843)),
        material=dark_graphite,
        name="bearing_strut_0",
    )
    body.visual(
        Box((0.006, 0.020, 0.050)),
        origin=Origin(xyz=(-0.010, 0.0, neck_height + 0.843)),
        material=dark_graphite,
        name="bearing_strut_1",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.0025),
        origin=Origin(xyz=(-0.036, 0.020, body_top_z + 0.00125)),
        material=soft_gray,
        name="dial_socket_0",
    )
    body.visual(
        Cylinder(radius=0.018, length=0.0025),
        origin=Origin(xyz=(0.036, 0.020, body_top_z + 0.00125)),
        material=soft_gray,
        name="dial_socket_1",
    )
    blower = model.part("blower_wheel")
    blower.visual(
        mesh_from_geometry(
            BlowerWheelGeometry(
                outer_radius=0.044,
                inner_radius=0.020,
                width=0.720,
                blade_count=30,
                blade_thickness=0.0022,
                blade_sweep_deg=28.0,
                backplate=True,
                shroud=True,
            ),
            "vertical_blower_wheel",
        ),
        material=dark_graphite,
        name="vertical_blower_wheel",
    )
    blower.visual(
        Cylinder(radius=0.005, length=0.730),
        material=satin_black,
        name="center_shaft",
    )
    blower.visual(
        Cylinder(radius=0.022, length=0.014),
        material=dark_graphite,
        name="shaft_hub",
    )

    dial_mesh_0 = mesh_from_geometry(
        KnobGeometry(
            0.034,
            0.019,
            body_style="cylindrical",
            edge_radius=0.001,
            grip=KnobGrip(style="knurled", count=34, depth=0.0008, helix_angle_deg=18.0),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "knurled_dial_0",
    )
    dial_mesh_1 = mesh_from_geometry(
        KnobGeometry(
            0.034,
            0.019,
            body_style="cylindrical",
            edge_radius=0.001,
            grip=KnobGrip(style="knurled", count=34, depth=0.0008, helix_angle_deg=-18.0),
            indicator=KnobIndicator(style="dot", mode="raised", angle_deg=90.0),
            center=False,
        ),
        "knurled_dial_1",
    )

    dial_0 = model.part("dial_0")
    dial_0.visual(dial_mesh_0, material=soft_gray, name="knurled_dial")

    dial_1 = model.part("dial_1")
    dial_1.visual(dial_mesh_1, material=soft_gray, name="knurled_dial")

    button = model.part("osc_button")
    button.visual(
        Cylinder(radius=0.014, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_graphite,
        name="button_cap",
    )
    button.visual(
        Cylinder(radius=0.007, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=dark_graphite,
        name="button_stem",
    )

    model.articulation(
        "base_to_body",
        ArticulationType.REVOLUTE,
        parent=base,
        child=body,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=-0.78, upper=0.78),
    )
    model.articulation(
        "body_to_blower",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=blower,
        origin=Origin(xyz=(0.0, 0.0, neck_height + 0.445)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=45.0),
    )
    model.articulation(
        "body_to_dial_0",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial_0,
        origin=Origin(xyz=(-0.036, 0.020, body_top_z + 0.0025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=10.0),
    )
    model.articulation(
        "body_to_dial_1",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=dial_1,
        origin=Origin(xyz=(0.036, 0.020, body_top_z + 0.0025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=10.0),
    )
    model.articulation(
        "body_to_osc_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(0.0, -0.030, body_top_z + 0.006)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=0.0, upper=0.006),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    def _dims(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])

    def _center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return ((lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0, (lo[2] + hi[2]) / 2.0)

    base = object_model.get_part("base")
    body = object_model.get_part("body")
    blower = object_model.get_part("blower_wheel")
    dial_0 = object_model.get_part("dial_0")
    dial_1 = object_model.get_part("dial_1")
    button = object_model.get_part("osc_button")

    oscillation = object_model.get_articulation("base_to_body")
    blower_spin = object_model.get_articulation("body_to_blower")
    dial_spin_0 = object_model.get_articulation("body_to_dial_0")
    dial_spin_1 = object_model.get_articulation("body_to_dial_1")
    button_press = object_model.get_articulation("body_to_osc_button")

    ctx.check(
        "primary joint types match the fan controls",
        oscillation.articulation_type == ArticulationType.REVOLUTE
        and blower_spin.articulation_type == ArticulationType.CONTINUOUS
        and dial_spin_0.articulation_type == ArticulationType.CONTINUOUS
        and dial_spin_1.articulation_type == ArticulationType.CONTINUOUS
        and button_press.articulation_type == ArticulationType.PRISMATIC,
    )
    ctx.check(
        "body oscillation has left-right limits",
        oscillation.motion_limits is not None
        and oscillation.motion_limits.lower is not None
        and oscillation.motion_limits.upper is not None
        and oscillation.motion_limits.lower < -0.7
        and oscillation.motion_limits.upper > 0.7,
    )
    ctx.check(
        "button travel is a short push stroke",
        button_press.motion_limits is not None
        and button_press.motion_limits.lower == 0.0
        and 0.004 <= (button_press.motion_limits.upper or 0.0) <= 0.008,
    )

    body_dims = _dims(ctx.part_world_aabb(body))
    ctx.check(
        "tower body is home-appliance tall and slim",
        body_dims is not None and body_dims[2] > 0.92 and body_dims[0] < 0.23 and body_dims[1] < 0.18,
        details=f"body_dims={body_dims}",
    )

    grille_dims = _dims(ctx.part_element_world_aabb(body, elem="front_grille"))
    ctx.check(
        "front grille runs nearly the full body height",
        body_dims is not None
        and grille_dims is not None
        and grille_dims[2] > 0.76
        and grille_dims[2] > 0.75 * body_dims[2],
        details=f"body_dims={body_dims}, grille_dims={grille_dims}",
    )

    ctx.expect_gap(
        body,
        base,
        axis="z",
        positive_elem="turntable_shadow",
        negative_elem="oscillation_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="oscillating body is seated on base bearing",
    )
    ctx.expect_gap(
        blower,
        body,
        axis="z",
        positive_elem="center_shaft",
        negative_elem="lower_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="blower shaft is supported by lower bearing cap",
    )
    ctx.expect_gap(
        dial_0,
        body,
        axis="z",
        positive_elem="knurled_dial",
        negative_elem="dial_socket_0",
        max_gap=0.001,
        max_penetration=0.0,
        name="first dial is seated on its top socket",
    )
    ctx.expect_gap(
        dial_1,
        body,
        axis="z",
        positive_elem="knurled_dial",
        negative_elem="dial_socket_1",
        max_gap=0.001,
        max_penetration=0.0,
        name="second dial is seated on its top socket",
    )
    ctx.expect_gap(
        button,
        body,
        axis="z",
        positive_elem="button_stem",
        negative_elem="top_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="oscillation button is supported on the top deck",
    )

    rest_grille = _center(ctx.part_element_world_aabb(body, elem="front_grille"))
    with ctx.pose({oscillation: 0.70}):
        turned_grille = _center(ctx.part_element_world_aabb(body, elem="front_grille"))
    ctx.check(
        "oscillation joint yaws the vertical body",
        rest_grille is not None
        and turned_grille is not None
        and turned_grille[0] > rest_grille[0] + 0.025,
        details=f"rest={rest_grille}, turned={turned_grille}",
    )

    rest_button = ctx.part_world_position(button)
    with ctx.pose({button_press: 0.006}):
        pressed_button = ctx.part_world_position(button)
    ctx.check(
        "oscillation button translates downward when pressed",
        rest_button is not None and pressed_button is not None and pressed_button[2] < rest_button[2] - 0.004,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    return ctx.report()


object_model = build_object_model()
