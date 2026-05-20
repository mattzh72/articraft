from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _extruded_prism_x(
    width: float, profile_yz: list[tuple[float, float]]
) -> MeshGeometry:
    """Return a simple solid prism extruded along X from a Y/Z side profile."""
    geom = MeshGeometry()
    half = width / 2.0
    n = len(profile_yz)

    left = [geom.add_vertex(-half, y, z) for y, z in profile_yz]
    right = [geom.add_vertex(half, y, z) for y, z in profile_yz]

    # Side walls.
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(left[i], left[j], right[j])
        geom.add_face(left[i], right[j], right[i])

    # End caps, triangulated around an interior centroid.
    cy = sum(y for y, _ in profile_yz) / n
    cz = sum(z for _, z in profile_yz) / n
    left_c = geom.add_vertex(-half, cy, cz)
    right_c = geom.add_vertex(half, cy, cz)
    for i in range(n):
        j = (i + 1) % n
        geom.add_face(left_c, left[i], left[j])
        geom.add_face(right_c, right[j], right[i])

    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="angled_glass_range_hood")

    stainless = model.material("brushed_stainless", rgba=(0.66, 0.67, 0.65, 1.0))
    dark_trim = model.material("black_glass_trim", rgba=(0.02, 0.025, 0.025, 1.0))
    smoked_glass = model.material("smoked_blue_glass", rgba=(0.28, 0.45, 0.55, 0.38))
    button_mat = model.material("satin_black_buttons", rgba=(0.015, 0.016, 0.017, 1.0))
    mark_mat = model.material("white_button_icons", rgba=(0.92, 0.92, 0.86, 1.0))

    canopy = model.part("canopy")

    # One continuous shallow wedge/canopy profile: rear wall, thin top, sloped
    # glass-backing face, and the lower control lip are all one metal shell.
    body_profile_yz = [
        (-0.420, 0.015),
        (-0.420, 0.060),
        (-0.305, 0.092),
        (-0.100, 0.500),
        (0.070, 0.500),
        (0.070, 0.310),
        (-0.220, 0.015),
    ]
    canopy.visual(
        mesh_from_geometry(_extruded_prism_x(0.92, body_profile_yz), "canopy_shell"),
        material=stainless,
        name="canopy_shell",
    )

    # Wall chimney duct sitting on and slightly into the shallow canopy top.
    canopy.visual(
        Box((0.32, 0.18, 0.54)),
        origin=Origin(xyz=(0.0, 0.000, 0.768)),
        material=stainless,
        name="chimney_duct",
    )
    canopy.visual(
        Box((0.36, 0.205, 0.018)),
        origin=Origin(xyz=(0.0, -0.010, 0.508)),
        material=stainless,
        name="duct_collar",
    )

    # Dark lower strip that the raised push-buttons sit proud of.
    canopy.visual(
        Box((0.56, 0.004, 0.046)),
        origin=Origin(xyz=(0.0, -0.422, 0.038)),
        material=dark_trim,
        name="control_strip",
    )

    # A slim hinge barrel along the top edge of the glass.  The glass leaf sits
    # just forward of this barrel, so the parts read connected without colliding.
    canopy.visual(
        Cylinder(radius=0.0123, length=0.82),
        origin=Origin(xyz=(0.0, -0.126, 0.465), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="hinge_barrel",
    )

    glass = model.part("glass_panel")
    glass_length = 0.440
    glass.visual(
        Box((0.86, 0.008, glass_length)),
        # Part frame is on the top hinge line; the sheet hangs down along local -Z.
        origin=Origin(xyz=(0.0, 0.0, -glass_length / 2.0)),
        material=smoked_glass,
        name="glass_sheet",
    )
    glass.visual(
        Box((0.88, 0.016, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=stainless,
        name="top_rail",
    )
    glass.visual(
        Box((0.78, 0.010, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -glass_length + 0.009)),
        material=dark_trim,
        name="bottom_trim",
    )

    model.articulation(
        "canopy_to_glass",
        ArticulationType.REVOLUTE,
        parent=canopy,
        child=glass,
        origin=Origin(xyz=(0.0, -0.140, 0.460), rpy=(-0.59, 0.0, 0.0)),
        # The panel is closed as a forward-sloped sheet; rotating about -X
        # raises the lower edge up and out, as on hinged angled-glass hoods.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.05),
    )

    # Four independent raised push buttons below the canopy.
    button_xs = (-0.180, -0.060, 0.060, 0.180)
    icon_widths = (0.020, 0.026, 0.026, 0.020)
    for idx, x in enumerate(button_xs):
        button = model.part(f"button_{idx}")
        button.visual(
            Box((0.074, 0.018, 0.028)),
            # The part frame is the mounting plane; the cap protrudes toward -Y.
            origin=Origin(xyz=(0.0, -0.009, 0.0)),
            material=button_mat,
            name="button_cap",
        )
        button.visual(
            Box((icon_widths[idx], 0.0015, 0.003)),
            origin=Origin(xyz=(0.0, -0.0186, 0.006)),
            material=mark_mat,
            name="button_icon",
        )
        model.articulation(
            f"control_to_button_{idx}",
            ArticulationType.PRISMATIC,
            parent=canopy,
            child=button,
            origin=Origin(xyz=(x, -0.424, 0.038)),
            # Positive travel is an inward press into the control strip.
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=0.08, lower=0.0, upper=0.006),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    canopy = object_model.get_part("canopy")
    glass = object_model.get_part("glass_panel")
    glass_joint = object_model.get_articulation("canopy_to_glass")

    ctx.allow_overlap(
        canopy,
        glass,
        elem_a="hinge_barrel",
        elem_b="top_rail",
        reason=(
            "The visible hinge barrel is intentionally seated into the moving "
            "top rail as a captured pin/knuckle interface."
        ),
    )

    # Closed glass is the sloped front skin, in front of and below the shallow
    # canopy body rather than a flat vertical placeholder.
    ctx.expect_overlap(
        glass,
        canopy,
        axes="x",
        elem_a="glass_sheet",
        elem_b="canopy_shell",
        min_overlap=0.80,
        name="glass spans most of canopy width",
    )
    ctx.expect_gap(
        glass,
        canopy,
        axis="z",
        positive_elem="glass_sheet",
        negative_elem="control_strip",
        min_gap=0.010,
        name="closed glass clears the lower control strip",
    )
    ctx.expect_overlap(
        canopy,
        glass,
        axes="xz",
        elem_a="hinge_barrel",
        elem_b="top_rail",
        min_overlap=0.008,
        name="top rail is captured on the hinge barrel",
    )

    rest_aabb = ctx.part_element_world_aabb(glass, elem="glass_sheet")
    with ctx.pose({glass_joint: 1.05}):
        open_aabb = ctx.part_element_world_aabb(glass, elem="glass_sheet")
    ctx.check(
        "glass hinge raises lower edge",
        rest_aabb is not None
        and open_aabb is not None
        and open_aabb[0][2] > rest_aabb[0][2] + 0.25,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )

    # Each button is a separate prismatic control mounted in a row on the lower
    # strip, with inward travel independent of its neighbors.
    for idx in range(4):
        button = object_model.get_part(f"button_{idx}")
        ctx.expect_gap(
            canopy,
            button,
            axis="y",
            positive_elem="control_strip",
            negative_elem="button_cap",
            max_gap=0.001,
            max_penetration=0.0,
            name=f"button {idx} sits proud on control strip",
        )
        ctx.expect_overlap(
            button,
            canopy,
            axes="xz",
            elem_a="button_cap",
            elem_b="control_strip",
            min_overlap=0.020,
            name=f"button {idx} lies within control strip",
        )

    middle = object_model.get_part("button_2")
    neighbor = object_model.get_part("button_1")
    middle_joint = object_model.get_articulation("control_to_button_2")
    middle_rest = ctx.part_world_position(middle)
    neighbor_rest = ctx.part_world_position(neighbor)
    with ctx.pose({middle_joint: 0.006}):
        middle_pressed = ctx.part_world_position(middle)
        neighbor_after = ctx.part_world_position(neighbor)
    ctx.check(
        "one button presses inward independently",
        middle_rest is not None
        and middle_pressed is not None
        and neighbor_rest is not None
        and neighbor_after is not None
        and middle_pressed[1] > middle_rest[1] + 0.005
        and abs(neighbor_after[1] - neighbor_rest[1]) < 1e-6,
        details=(
            f"middle_rest={middle_rest}, middle_pressed={middle_pressed}, "
            f"neighbor_rest={neighbor_rest}, neighbor_after={neighbor_after}"
        ),
    )

    return ctx.report()


object_model = build_object_model()
