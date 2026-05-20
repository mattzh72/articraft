from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _mesh(geometry, name: str):
    return mesh_from_geometry(geometry, name)


def _rounded_plate(length: float, width: float, thickness: float, radius: float):
    return ExtrudeGeometry(
        rounded_rect_profile(length, width, radius, corner_segments=8),
        thickness,
        center=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="air_mouse_remote")

    satin_black = model.material("satin_black", rgba=(0.025, 0.026, 0.030, 1.0))
    soft_graphite = model.material("soft_graphite", rgba=(0.105, 0.112, 0.120, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.010, 0.010, 0.012, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.22, 0.23, 0.24, 1.0))
    cool_grey = model.material("cool_grey", rgba=(0.36, 0.38, 0.40, 1.0))
    glass_black = model.material("glass_black", rgba=(0.004, 0.004, 0.006, 1.0))
    status_green = model.material("status_green", rgba=(0.10, 0.90, 0.42, 1.0))

    body = model.part("body")

    # A hand-sized air-mouse body: an elongated capsule, non-uniformly scaled so
    # its cross-section is a soft oval rather than a round tube.
    body_shell = CapsuleGeometry(radius=0.5, length=1.15, radial_segments=56, height_segments=12)
    body_shell.scale(0.026, 0.048, 0.0744).rotate_y(pi / 2.0)
    body.visual(
        _mesh(body_shell, "body_shell"),
        origin=Origin(),
        material=satin_black,
        name="body_shell",
    )

    top_panel = _rounded_plate(0.102, 0.034, 0.0016, 0.014)
    body.visual(
        _mesh(top_panel, "top_panel"),
        origin=Origin(xyz=(0.004, 0.0, 0.0134)),
        material=soft_graphite,
        name="top_panel",
    )

    # Two raised cheeks form the cradle that carries the horizontal jog-wheel
    # axle.  They are part of the housing and overlap the top shell slightly, as
    # molded plastic bosses would.
    for y in (-0.019, 0.019):
        body.visual(
            Box((0.038, 0.007, 0.020)),
            origin=Origin(xyz=(0.030, y, 0.022)),
            material=soft_graphite,
            name=f"jog_cheek_{0 if y < 0 else 1}",
        )
        body.visual(
            Cylinder(radius=0.0048, length=0.003),
            origin=Origin(xyz=(0.030, y, 0.0285), rpy=(pi / 2.0, 0.0, 0.0)),
            material=cool_grey,
            name=f"axle_cap_{0 if y < 0 else 1}",
        )

    # Front IR/optical window and a tiny status light make the object read as a
    # handheld remote rather than a generic computer mouse.
    body.visual(
        Cylinder(radius=0.009, length=0.003),
        origin=Origin(xyz=(0.0805, 0.0, 0.001), rpy=(0.0, pi / 2.0, 0.0)),
        material=glass_black,
        name="front_window",
    )
    body.visual(
        Cylinder(radius=0.0024, length=0.0012),
        origin=Origin(xyz=(-0.025, 0.0, 0.0148)),
        material=status_green,
        name="status_light",
    )

    # Exposed hinge knuckles fixed to the underside of the body.  The center
    # knuckle belongs to the moving cover and sits in the gap between these two.
    for y in (-0.018, 0.018):
        body.visual(
            Box((0.010, 0.006, 0.008)),
            origin=Origin(xyz=(-0.047, y, -0.0115)),
            material=dark_grey,
            name=f"cover_hinge_web_{0 if y < 0 else 1}",
        )
        body.visual(
            Cylinder(radius=0.0040, length=0.010),
            origin=Origin(xyz=(-0.047, y, -0.0155), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_grey,
            name=f"cover_hinge_lug_{0 if y < 0 else 1}",
        )

    jog_dial = model.part("jog_dial")
    jog_geom = KnobGeometry(
        0.029,
        0.023,
        body_style="cylindrical",
        edge_radius=0.0012,
        grip=KnobGrip(style="ribbed", count=28, depth=0.0009, width=0.0012),
    )
    jog_dial.visual(
        _mesh(jog_geom, "jog_wheel"),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber_black,
        name="wheel_tread",
    )
    jog_dial.visual(
        Cylinder(radius=0.006, length=0.032),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=cool_grey,
        name="wheel_hub",
    )

    battery_cover = model.part("battery_cover")
    battery_cover.visual(
        _mesh(_rounded_plate(0.088, 0.030, 0.0030, 0.008), "battery_cover_panel"),
        # The child frame is on the hinge line at the cover's short edge.  The
        # closed cover extends along local +X.
        origin=Origin(xyz=(0.044, 0.0, 0.0007)),
        material=dark_grey,
        name="cover_panel",
    )
    battery_cover.visual(
        Cylinder(radius=0.0035, length=0.027),
        origin=Origin(xyz=(0.0, 0.0, -0.0016), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_grey,
        name="cover_hinge_barrel",
    )
    battery_cover.visual(
        Box((0.014, 0.018, 0.0012)),
        origin=Origin(xyz=(0.073, 0.0, -0.0010)),
        material=cool_grey,
        name="finger_latch",
    )

    model.articulation(
        "body_to_jog_dial",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=jog_dial,
        origin=Origin(xyz=(0.030, 0.0, 0.0290)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.08, velocity=30.0),
    )
    model.articulation(
        "body_to_battery_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=battery_cover,
        origin=Origin(xyz=(-0.047, 0.0, -0.0155)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.35, velocity=2.5, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    jog_dial = object_model.get_part("jog_dial")
    battery_cover = object_model.get_part("battery_cover")
    jog_joint = object_model.get_articulation("body_to_jog_dial")
    cover_joint = object_model.get_articulation("body_to_battery_cover")

    ctx.check(
        "jog dial is a continuous horizontal-axis wheel",
        jog_joint.articulation_type == ArticulationType.CONTINUOUS and tuple(jog_joint.axis) == (0.0, 1.0, 0.0),
        details=f"type={jog_joint.articulation_type}, axis={jog_joint.axis}",
    )
    ctx.check(
        "battery cover has a realistic one-way hinge range",
        cover_joint.motion_limits is not None
        and cover_joint.motion_limits.lower == 0.0
        and 1.0 <= cover_joint.motion_limits.upper <= 1.4,
        details=f"limits={cover_joint.motion_limits}",
    )
    for cheek_name in ("jog_cheek_0", "jog_cheek_1"):
        ctx.allow_overlap(
            body,
            jog_dial,
            elem_a=cheek_name,
            elem_b="wheel_hub",
            reason="The jog wheel axle is intentionally captured inside the cradle cheek bearing.",
        )
        ctx.expect_overlap(
            body,
            jog_dial,
            axes="y",
            elem_a=cheek_name,
            elem_b="wheel_hub",
            min_overlap=0.0002,
            name=f"{cheek_name} captures the jog axle end",
        )

    ctx.expect_within(
        battery_cover,
        body,
        axes="xy",
        inner_elem="cover_panel",
        outer_elem="body_shell",
        margin=0.002,
        name="battery cover sits within the underside footprint",
    )
    ctx.expect_gap(
        body,
        battery_cover,
        axis="z",
        positive_elem="body_shell",
        negative_elem="cover_panel",
        min_gap=0.0,
        max_gap=0.006,
        name="closed battery cover is just below the underside",
    )
    ctx.expect_gap(
        jog_dial,
        body,
        axis="z",
        positive_elem="wheel_tread",
        negative_elem="top_panel",
        min_gap=0.0,
        max_gap=0.004,
        name="jog dial clears the top face",
    )

    rest_aabb = ctx.part_element_world_aabb(battery_cover, elem="cover_panel")
    with ctx.pose({cover_joint: 1.10}):
        open_aabb = ctx.part_element_world_aabb(battery_cover, elem="cover_panel")
    ctx.check(
        "battery cover pivots downward from its short-edge hinge",
        rest_aabb is not None and open_aabb is not None and open_aabb[0][2] < rest_aabb[0][2] - 0.030,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
