from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)
import cadquery as cq


PENDANT_XS = (-0.92, -0.46, 0.0, 0.46, 0.92)
GRIP_NAMES = (
    "fixed_grip_0",
    "fixed_grip_1",
    "fixed_grip_2",
    "fixed_grip_3",
    "fixed_grip_4",
)
SLIDE_TRAVEL = 0.18
GRIP_BOTTOM_Z = -0.055
CORD_BOTTOM_Z = -0.82


def _shade_shell_geometry():
    """Thin open spun shade, modeled as a real hollow lathed shell."""
    return LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.042, -0.032),
            (0.058, -0.050),
            (0.100, -0.150),
            (0.178, -0.290),
        ],
        inner_profile=[
            (0.033, -0.041),
            (0.049, -0.058),
            (0.089, -0.154),
            (0.166, -0.278),
        ],
        segments=72,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )


def _grip_sleeve_geometry():
    """Short hollow cable-grip sleeve with a clear center bore for the cord."""
    return LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.031, -0.055),
            (0.036, -0.050),
            (0.036, -0.005),
            (0.031, 0.000),
        ],
        inner_profile=[
            (0.012, -0.052),
            (0.012, -0.003),
        ],
        segments=48,
        start_cap="round",
        end_cap="round",
        lip_samples=5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="kitchen_island_pendant_track")

    matte_black = model.material("matte_black", rgba=(0.015, 0.014, 0.013, 1.0))
    dark_channel = model.material("shadow_channel", rgba=(0.003, 0.003, 0.003, 1.0))
    warm_brass = model.material("warm_brass", rgba=(0.86, 0.62, 0.28, 1.0))
    shade_finish = model.material("champagne_shade", rgba=(0.72, 0.55, 0.36, 1.0))
    porcelain = model.material("porcelain_socket", rgba=(0.93, 0.88, 0.76, 1.0))
    warm_glass = model.material("warm_glass", rgba=(1.0, 0.82, 0.42, 0.72))
    ceiling_paint = model.material("ceiling_paint", rgba=(0.94, 0.92, 0.86, 1.0))

    shade_shell = mesh_from_geometry(_shade_shell_geometry(), "spun_cone_shade")
    grip_sleeve = mesh_from_geometry(_grip_sleeve_geometry(), "hollow_prismatic_grip")

    rail = model.part("ceiling_rail")
    rail.visual(
        Box((2.64, 0.28, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        material=ceiling_paint,
        name="ceiling_pad",
    )
    # The rail is a long hollow rectangular track: side walls and a top web leave
    # a central bottom slot so the pendant cords can slide without intersecting
    # a solid proxy.
    rail.visual(
        Box((2.46, 0.16, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.1625)),
        material=matte_black,
        name="top_web",
    )
    rail.visual(
        Box((2.46, 0.040, 0.145)),
        origin=Origin(xyz=(0.0, 0.058, 0.0725)),
        material=matte_black,
        name="side_wall_0",
    )
    rail.visual(
        Box((2.46, 0.040, 0.145)),
        origin=Origin(xyz=(0.0, -0.058, 0.0725)),
        material=matte_black,
        name="side_wall_1",
    )
    rail.visual(
        Box((0.045, 0.16, 0.145)),
        origin=Origin(xyz=(-1.2525, 0.0, 0.0725)),
        material=matte_black,
        name="end_cap_0",
    )
    rail.visual(
        Box((0.045, 0.16, 0.145)),
        origin=Origin(xyz=(1.2525, 0.0, 0.0725)),
        material=matte_black,
        name="end_cap_1",
    )
    rail.visual(
        Box((2.30, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, 0.034, -0.004)),
        material=dark_channel,
        name="bottom_lip_0",
    )
    rail.visual(
        Box((2.30, 0.008, 0.008)),
        origin=Origin(xyz=(0.0, -0.034, -0.004)),
        material=dark_channel,
        name="bottom_lip_1",
    )

    for index, x in enumerate(PENDANT_XS):
        rail.visual(
            grip_sleeve,
            origin=Origin(xyz=(x, 0.0, 0.0)),
            material=warm_brass,
            name=GRIP_NAMES[index],
        )

    for index, x in enumerate(PENDANT_XS):
        cord = model.part(f"cord_{index}")
        cord.visual(
            Cylinder(radius=0.005, length=1.020),
            origin=Origin(xyz=(0.0, 0.0, -0.310)),
            material=matte_black,
            name="cable",
        )
        cord.visual(
            Cylinder(radius=0.018, length=0.026),
            origin=Origin(xyz=(0.0, 0.0, -0.075)),
            material=warm_brass,
            name="sliding_stop",
        )
        cord.visual(
            Cylinder(radius=0.014, length=0.034),
            origin=Origin(xyz=(0.0, 0.0, CORD_BOTTOM_Z + 0.017)),
            material=warm_brass,
            name="lower_ferrule",
        )

        model.articulation(
            f"cord_slide_{index}",
            ArticulationType.PRISMATIC,
            parent=rail,
            child=cord,
            origin=Origin(xyz=(x, 0.0, GRIP_BOTTOM_Z)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=35.0,
                velocity=0.18,
                lower=0.0,
                upper=SLIDE_TRAVEL,
            ),
        )

        shade = model.part(f"shade_{index}")
        shade.visual(
            shade_shell,
            origin=Origin(),
            material=shade_finish,
            name="shade_shell",
        )
        shade.visual(
            Cylinder(radius=0.043, length=0.055),
            origin=Origin(xyz=(0.0, 0.0, -0.0275)),
            material=warm_brass,
            name="swivel_collar",
        )
        shade.visual(
            Cylinder(radius=0.034, length=0.045),
            origin=Origin(xyz=(0.0, 0.0, -0.070)),
            material=porcelain,
            name="socket",
        )
        shade.visual(
            Sphere(radius=0.042),
            origin=Origin(xyz=(0.0, 0.0, -0.130)),
            material=warm_glass,
            name="bulb",
        )

        model.articulation(
            f"shade_swivel_{index}",
            ArticulationType.REVOLUTE,
            parent=cord,
            child=shade,
            origin=Origin(xyz=(0.0, 0.0, CORD_BOTTOM_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=1.2,
                lower=-0.45,
                upper=0.45,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    slides = [object_model.get_articulation(f"cord_slide_{i}") for i in range(5)]
    swivels = [object_model.get_articulation(f"shade_swivel_{i}") for i in range(5)]

    ctx.check(
        "five prismatic cord length grips",
        all(j.articulation_type == ArticulationType.PRISMATIC for j in slides)
        and all(
            j.motion_limits is not None
            and j.motion_limits.lower == 0.0
            and j.motion_limits.upper == SLIDE_TRAVEL
            for j in slides
        ),
        details="Every pendant cord should have a bounded prismatic length adjustment.",
    )
    ctx.check(
        "five revolute shade swivels",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in swivels)
        and all(
            j.motion_limits is not None
            and j.motion_limits.lower < 0.0
            and j.motion_limits.upper > 0.0
            for j in swivels
        ),
        details="Every shade should rotate at the cord end on a revolute joint.",
    )

    rail = object_model.get_part("ceiling_rail")
    center_cord = object_model.get_part("cord_2")
    center_shade = object_model.get_part("shade_2")
    center_slide = object_model.get_articulation("cord_slide_2")
    center_swivel = object_model.get_articulation("shade_swivel_2")

    ctx.expect_within(
        center_cord,
        rail,
        axes="xy",
        inner_elem="cable",
        outer_elem="fixed_grip_2",
        margin=0.002,
        name="cord stays centered in its rail grip",
    )
    ctx.expect_overlap(
        center_cord,
        rail,
        axes="z",
        elem_a="cable",
        elem_b="fixed_grip_2",
        min_overlap=0.045,
        name="shortened cord remains captured by grip",
    )

    rest_shade_pos = ctx.part_world_position(center_shade)
    with ctx.pose({center_slide: SLIDE_TRAVEL}):
        ctx.expect_within(
            center_cord,
            rail,
            axes="xy",
            inner_elem="cable",
            outer_elem="fixed_grip_2",
            margin=0.002,
            name="extended cord remains centered in its grip",
        )
        ctx.expect_overlap(
            center_cord,
            rail,
            axes="z",
            elem_a="cable",
            elem_b="fixed_grip_2",
            min_overlap=0.015,
            name="extended cord retains insertion in grip",
        )
        extended_shade_pos = ctx.part_world_position(center_shade)

    ctx.check(
        "prismatic grip lowers pendant shade",
        rest_shade_pos is not None
        and extended_shade_pos is not None
        and extended_shade_pos[2] < rest_shade_pos[2] - 0.15,
        details=f"rest={rest_shade_pos}, extended={extended_shade_pos}",
    )

    rest_shell_aabb = ctx.part_element_world_aabb(center_shade, elem="shade_shell")
    with ctx.pose({center_swivel: 0.40}):
        tilted_shell_aabb = ctx.part_element_world_aabb(center_shade, elem="shade_shell")

    if rest_shell_aabb is not None and tilted_shell_aabb is not None:
        rest_x = (rest_shell_aabb[0][0] + rest_shell_aabb[1][0]) * 0.5
        tilted_x = (tilted_shell_aabb[0][0] + tilted_shell_aabb[1][0]) * 0.5
        swivel_moves = abs(tilted_x - rest_x) > 0.035
    else:
        swivel_moves = False
    ctx.check(
        "revolute shade visibly tilts",
        swivel_moves,
        details=f"rest_aabb={rest_shell_aabb}, tilted_aabb={tilted_shell_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
