from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_slab(width: float, height: float, depth: float, radius: float):
    """Rounded X/Z slab extruded along Y."""
    geom = ExtrudeGeometry(
        rounded_rect_profile(width, height, radius, corner_segments=8),
        depth,
        cap=True,
        center=True,
    )
    return geom.rotate_x(-pi / 2.0)


def _rounded_frame(
    width: float,
    height: float,
    inner_width: float,
    inner_height: float,
    depth: float,
    outer_radius: float,
    inner_radius: float,
):
    """Continuous rounded-rectangle frame in the X/Z plane, extruded along Y."""
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(width, height, outer_radius, corner_segments=10),
        [rounded_rect_profile(inner_width, inner_height, inner_radius, corner_segments=8)],
        depth,
        cap=True,
        center=True,
    )
    return geom.rotate_x(-pi / 2.0)


def _rear_support_panel(width: float, height: float, depth: float):
    """Wide one-piece kickstand frame with its hinge edge on local z=0."""
    geom = ExtrudeWithHolesGeometry(
        rounded_rect_profile(width, height, 0.006, corner_segments=8),
        [rounded_rect_profile(width - 0.022, height - 0.036, 0.004, corner_segments=6)],
        depth,
        cap=True,
        center=True,
    )
    geom.rotate_x(-pi / 2.0)
    # Place the inner face just behind the hinge line and the lower edge on z=0.
    return geom.translate(0.0, depth / 2.0, height / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_industrial_multimeter")

    graphite = model.material("dark_graphite_polymer", rgba=(0.08, 0.085, 0.09, 1.0))
    charcoal = model.material("charcoal_face", rgba=(0.015, 0.017, 0.018, 1.0))
    rubber = model.material("safety_orange_rubber", rgba=(0.95, 0.46, 0.05, 1.0))
    glass = model.material("smoked_lcd_glass", rgba=(0.05, 0.10, 0.11, 0.78))
    button_mat = model.material("matte_soft_key_gray", rgba=(0.36, 0.38, 0.38, 1.0))
    black = model.material("black_rubber", rgba=(0.005, 0.005, 0.005, 1.0))

    case = model.part("case")

    case.visual(
        mesh_from_geometry(_rounded_slab(0.090, 0.180, 0.034, 0.010), "case_body"),
        material=graphite,
        name="case_body",
    )
    # Thick rubber overmold frames wrap the front and rear corners of the square
    # handheld housing.
    case.visual(
        mesh_from_geometry(
            _rounded_frame(0.104, 0.195, 0.074, 0.155, 0.008, 0.018, 0.008),
            "front_overmold",
        ),
        origin=Origin(xyz=(0.0, -0.0182, 0.0)),
        material=rubber,
        name="front_overmold",
    )
    case.visual(
        mesh_from_geometry(
            _rounded_frame(0.104, 0.195, 0.074, 0.155, 0.008, 0.018, 0.008),
            "rear_overmold",
        ),
        origin=Origin(xyz=(0.0, 0.0182, 0.0)),
        material=rubber,
        name="rear_overmold",
    )
    for x in (-0.049, 0.049):
        case.visual(
            Box((0.012, 0.044, 0.150)),
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=rubber,
            name=f"side_overmold_{'n' if x < 0 else 'p'}",
        )
    for z in (-0.092, 0.092):
        case.visual(
            Box((0.080, 0.044, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=rubber,
            name=f"end_overmold_{'low' if z < 0 else 'high'}",
        )

    # Continuous front instrument face, display window, and selector scale.
    case.visual(
        Box((0.070, 0.002, 0.130)),
        origin=Origin(xyz=(0.0, -0.0192, 0.006)),
        material=charcoal,
        name="front_face",
    )
    case.visual(
        mesh_from_geometry(
            _rounded_frame(0.072, 0.044, 0.061, 0.032, 0.002, 0.006, 0.003),
            "display_bezel",
        ),
        origin=Origin(xyz=(0.0, -0.0212, 0.055)),
        material=black,
        name="display_bezel",
    )
    case.visual(
        Box((0.058, 0.001, 0.030)),
        origin=Origin(xyz=(0.0, -0.0227, 0.055)),
        material=glass,
        name="display_window",
    )
    case.visual(
        Cylinder(radius=0.034, length=0.0016),
        origin=Origin(xyz=(0.0, -0.0210, -0.033), rpy=(pi / 2.0, 0.0, 0.0)),
        material=charcoal,
        name="selector_scale_plate",
    )
    # Two fixed lower pivot knuckles on the rear case, matching the kickstand.
    for x in (-0.040, 0.040):
        case.visual(
            Cylinder(radius=0.0045, length=0.009),
            origin=Origin(xyz=(x, 0.023, -0.070), rpy=(0.0, pi / 2.0, 0.0)),
            material=black,
            name=f"rear_pivot_{'n' if x < 0 else 'p'}",
        )

    selector = model.part("selector")
    selector.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.044,
                0.016,
                body_style="cylindrical",
                top_diameter=0.040,
                edge_radius=0.0008,
                grip=KnobGrip(
                    style="knurled",
                    count=44,
                    depth=0.0012,
                    helix_angle_deg=22.0,
                ),
                indicator=KnobIndicator(style="line", mode="raised", angle_deg=90.0),
                center=False,
            ),
            "selector_knob",
        ),
        material=black,
        name="selector_knob",
    )
    model.articulation(
        "case_to_selector",
        ArticulationType.CONTINUOUS,
        parent=case,
        child=selector,
        origin=Origin(xyz=(0.0, -0.0218, -0.033), rpy=(pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=6.0),
    )

    key_xs = (-0.032, -0.016, 0.0, 0.016, 0.032)
    for index, x in enumerate(key_xs):
        key = model.part(f"soft_key_{index}")
        key.visual(
            Box((0.012, 0.005, 0.008)),
            origin=Origin(xyz=(0.0, -0.0025, 0.0)),
            material=button_mat,
            name="key_cap",
        )
        model.articulation(
            f"case_to_soft_key_{index}",
            ArticulationType.PRISMATIC,
            parent=case,
            child=key,
            origin=Origin(xyz=(x, -0.0202, 0.019)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.03, lower=0.0, upper=0.0025),
        )

    rear_support = model.part("rear_support")
    rear_support.visual(
        mesh_from_geometry(_rear_support_panel(0.070, 0.112, 0.004), "rear_support_panel"),
        material=black,
        name="support_panel",
    )
    rear_support.visual(
        Cylinder(radius=0.0035, length=0.066),
        origin=Origin(xyz=(0.0, 0.002, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=black,
        name="support_hinge_bar",
    )
    model.articulation(
        "case_to_rear_support",
        ArticulationType.REVOLUTE,
        parent=case,
        child=rear_support,
        origin=Origin(xyz=(0.0, 0.023, -0.070)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.5, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    case = object_model.get_part("case")
    selector = object_model.get_part("selector")
    support = object_model.get_part("rear_support")

    ctx.expect_contact(
        "soft_key_2",
        case,
        elem_a="key_cap",
        elem_b="front_face",
        contact_tol=0.0002,
        name="middle soft key sits on the front face",
    )
    ctx.expect_overlap(
        "soft_key_0",
        "soft_key_4",
        axes="z",
        min_overlap=0.007,
        elem_a="key_cap",
        elem_b="key_cap",
        name="soft keys share a neat row",
    )
    ctx.expect_contact(
        selector,
        case,
        elem_a="selector_knob",
        elem_b="selector_scale_plate",
        contact_tol=0.0005,
        name="selector knob is seated on its scale plate",
    )
    ctx.expect_gap(
        support,
        case,
        axis="y",
        min_gap=0.0002,
        max_gap=0.008,
        positive_elem="support_panel",
        negative_elem="rear_overmold",
        name="rear support stows just behind the back panel",
    )

    support_joint = object_model.get_articulation("case_to_rear_support")
    closed_pos = ctx.part_world_position(support)
    with ctx.pose({support_joint: 0.85}):
        open_pos = ctx.part_world_position(support)
        ctx.expect_gap(
            support,
            case,
            axis="y",
            min_gap=0.004,
            positive_elem="support_panel",
            negative_elem="case_body",
            name="opened rear support folds away from case",
        )
    ctx.check(
        "rear support joint is a lower pivot",
        closed_pos is not None and open_pos is not None and abs(open_pos[2] - closed_pos[2]) < 0.001,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


object_model = build_object_model()
