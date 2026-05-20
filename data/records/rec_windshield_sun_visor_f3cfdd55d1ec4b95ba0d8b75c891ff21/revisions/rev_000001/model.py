from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    LoftGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


PANEL_WIDTH = 0.50
PANEL_HEIGHT = 0.19
PANEL_THICKNESS = 0.026
HINGE_X = 0.25
HINGE_Z = -0.052


def _padded_panel_mesh():
    """Soft, rounded rectangular visor cushion centered on its own face."""

    def section(width: float, height: float, radius: float, z: float):
        return [(x, y, z) for x, y in rounded_rect_profile(width, height, radius, corner_segments=10)]

    panel = LoftGeometry(
        [
            section(PANEL_WIDTH - 0.010, PANEL_HEIGHT - 0.008, 0.034, -PANEL_THICKNESS * 0.50),
            section(PANEL_WIDTH, PANEL_HEIGHT, 0.040, 0.0),
            section(PANEL_WIDTH - 0.010, PANEL_HEIGHT - 0.008, 0.034, PANEL_THICKNESS * 0.50),
        ],
        cap=True,
        closed=True,
    )
    # Rotate the XY rounded rectangle so its height is vertical (local Z) and
    # its thickness is front-to-back (local Y) on the visor.
    return panel.rotate_x(-pi / 2.0)


def _outer_frame_mesh():
    """A simple light plastic perimeter frame, kept as one ring mesh."""

    frame = ExtrudeWithHolesGeometry(
        rounded_rect_profile(PANEL_WIDTH + 0.024, PANEL_HEIGHT + 0.024, 0.046, corner_segments=10),
        [rounded_rect_profile(PANEL_WIDTH - 0.026, PANEL_HEIGHT - 0.032, 0.030, corner_segments=10)],
        height=0.007,
        center=True,
    )
    # ExtrudeWithHolesGeometry extrudes in local Z.  Rotate so the thin frame
    # thickness sits proud of the visor face along local Y.
    return frame.rotate_x(-pi / 2.0)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="car_windshield_sun_visor")

    warm_headliner = model.material("warm_headliner", rgba=(0.74, 0.70, 0.63, 1.0))
    visor_fabric = model.material("visor_fabric", rgba=(0.78, 0.74, 0.66, 1.0))
    seam_plastic = model.material("seam_plastic", rgba=(0.62, 0.58, 0.51, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.68, 0.69, 0.68, 1.0))
    dark_shadow = model.material("dark_shadow", rgba=(0.14, 0.13, 0.12, 1.0))
    mirror_glass = model.material("mirror_glass", rgba=(0.54, 0.62, 0.64, 0.55))

    bracket = model.part("roof_bracket")
    bracket.visual(
        Box((0.155, 0.092, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=warm_headliner,
        name="mount_plate",
    )
    bracket.visual(
        Box((0.105, 0.064, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=seam_plastic,
        name="pivot_housing",
    )
    bracket.visual(
        Cylinder(radius=0.026, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=seam_plastic,
        name="pivot_boss",
    )
    for screw_x in (-0.043, 0.043):
        bracket.visual(
            Cylinder(radius=0.010, length=0.004),
            origin=Origin(xyz=(screw_x, 0.0, 0.035)),
            material=satin_metal,
            name=f"screw_{0 if screw_x < 0 else 1}",
        )
    bracket.inertial = Inertial.from_geometry(
        Box((0.155, 0.092, 0.055)),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )

    swing_arm = model.part("swing_arm")
    swing_arm.visual(
        Cylinder(radius=0.013, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=satin_metal,
        name="vertical_pin",
    )
    swing_arm.visual(
        Cylinder(radius=0.024, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=seam_plastic,
        name="pivot_collar",
    )
    swing_arm.visual(
        Sphere(radius=0.018),
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        material=seam_plastic,
        name="elbow_knuckle",
    )
    swing_arm.visual(
        Cylinder(radius=0.0075, length=0.515),
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_metal,
        name="hinge_spindle",
    )
    swing_arm.visual(
        Cylinder(radius=0.014, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_shadow,
        name="side_stop",
    )
    swing_arm.inertial = Inertial.from_geometry(
        Box((0.53, 0.048, 0.070)),
        mass=0.18,
        origin=Origin(xyz=(0.18, 0.0, -0.030)),
    )

    visor = model.part("visor_panel")
    visor.visual(
        mesh_from_geometry(_padded_panel_mesh(), "visor_padded_panel"),
        origin=Origin(xyz=(0.0, 0.0, -0.123)),
        material=visor_fabric,
        name="padded_panel",
    )
    visor.visual(
        mesh_from_geometry(_outer_frame_mesh(), "visor_outer_frame"),
        origin=Origin(xyz=(0.0, -0.015, -0.123)),
        material=seam_plastic,
        name="outer_frame",
    )
    visor.visual(
        Cylinder(radius=0.014, length=0.465),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=seam_plastic,
        name="top_sleeve",
    )
    visor.visual(
        Box((0.440, 0.020, 0.020)),
        origin=Origin(xyz=(0.028, 0.0, -0.021)),
        material=seam_plastic,
        name="top_bridge",
    )
    visor.visual(
        Box((0.155, 0.008, 0.055)),
        origin=Origin(xyz=(0.070, -0.016, -0.112)),
        material=mirror_glass,
        name="vanity_mirror",
    )
    visor.visual(
        Box((0.174, 0.006, 0.074)),
        origin=Origin(xyz=(0.070, -0.015, -0.112)),
        material=seam_plastic,
        name="mirror_bezel",
    )
    visor.visual(
        Box((0.166, 0.004, 0.062)),
        origin=Origin(xyz=(0.070, -0.019, -0.112)),
        material=mirror_glass,
        name="mirror_face",
    )
    visor.inertial = Inertial.from_geometry(
        Box((PANEL_WIDTH, PANEL_THICKNESS, PANEL_HEIGHT + 0.034)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, -0.112)),
    )

    model.articulation(
        "side_swing",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=swing_arm,
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.55),
    )
    model.articulation(
        "visor_flip",
        ArticulationType.REVOLUTE,
        parent=swing_arm,
        child=visor,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.4, lower=-1.35, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("roof_bracket")
    swing_arm = object_model.get_part("swing_arm")
    visor = object_model.get_part("visor_panel")
    side_swing = object_model.get_articulation("side_swing")
    visor_flip = object_model.get_articulation("visor_flip")

    ctx.allow_overlap(
        swing_arm,
        visor,
        elem_a="hinge_spindle",
        elem_b="top_sleeve",
        reason="The metal hinge spindle is intentionally captured inside the visor's molded sleeve.",
    )
    ctx.expect_within(
        swing_arm,
        visor,
        axes="yz",
        inner_elem="hinge_spindle",
        outer_elem="top_sleeve",
        margin=0.001,
        name="spindle centered inside sleeve",
    )
    ctx.expect_overlap(
        swing_arm,
        visor,
        axes="x",
        elem_a="hinge_spindle",
        elem_b="top_sleeve",
        min_overlap=0.45,
        name="spindle retained along visor width",
    )
    ctx.expect_gap(
        bracket,
        swing_arm,
        axis="z",
        positive_elem="pivot_boss",
        negative_elem="pivot_collar",
        max_gap=0.002,
        max_penetration=0.001,
        name="side pivot collar seats against boss",
    )

    rest_visor_pos = ctx.part_world_position(visor)
    with ctx.pose({side_swing: 1.30}):
        side_visor_pos = ctx.part_world_position(visor)
    ctx.check(
        "vertical swing pivot moves visor sideways",
        rest_visor_pos is not None
        and side_visor_pos is not None
        and side_visor_pos[1] > rest_visor_pos[1] + 0.18,
        details=f"rest={rest_visor_pos}, side={side_visor_pos}",
    )

    down_aabb = ctx.part_world_aabb(visor)
    with ctx.pose({visor_flip: -1.25}):
        stowed_aabb = ctx.part_world_aabb(visor)
    ctx.check(
        "horizontal hinge flips visor up toward roof",
        down_aabb is not None
        and stowed_aabb is not None
        and float(stowed_aabb[0][2]) > float(down_aabb[0][2]) + 0.06,
        details=f"down={down_aabb}, stowed={stowed_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
