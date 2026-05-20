from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _shell_mesh(
    name: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_kit_zoom_lens")

    matte_black = model.material("matte_black", rgba=(0.12, 0.12, 0.13, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.09, 0.09, 0.10, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.21, 0.22, 0.24, 1.0))
    mount_metal = model.material("mount_metal", rgba=(0.69, 0.71, 0.74, 1.0))
    coated_glass = model.material("coated_glass", rgba=(0.20, 0.34, 0.38, 0.35))

    outer_barrel = model.part("outer_barrel")
    outer_barrel.visual(
        _shell_mesh(
            "outer_barrel_shell",
            [
                (0.0315, 0.0000),
                (0.0315, 0.0030),
                (0.0303, 0.0040),
                (0.0303, 0.0105),
                (0.0282, 0.0112),
                (0.0282, 0.0123),
                (0.0274, 0.0129),
                (0.0274, 0.0271),
                (0.0282, 0.0277),
                (0.0282, 0.0288),
                (0.0292, 0.0295),
                (0.0292, 0.0396),
                (0.0280, 0.0406),
                (0.0280, 0.0470),
            ],
            [
                (0.0210, 0.0000),
                (0.0210, 0.0030),
                (0.0245, 0.0065),
                (0.0245, 0.0390),
                (0.0253, 0.0470),
            ],
        ),
        material=matte_black,
        name="outer_shell",
    )
    outer_barrel.visual(
        _shell_mesh(
            "mount_ring_shell",
            [
                (0.0318, 0.0000),
                (0.0318, 0.0018),
            ],
            [
                (0.0212, 0.0000),
                (0.0212, 0.0018),
            ],
        ),
        material=mount_metal,
        name="mount_ring",
    )
    outer_barrel.visual(
        Cylinder(radius=0.0212, length=0.0050),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        material=satin_graphite,
        name="rear_baffle",
    )
    outer_barrel.inertial = Inertial.from_geometry(
        Box((0.064, 0.064, 0.050)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
    )

    zoom_ring = model.part("zoom_ring")
    zoom_ring.visual(
        _shell_mesh(
            "zoom_ring_shell",
            [
                (0.0318, -0.0065),
                (0.0323, -0.0050),
                (0.0319, -0.0035),
                (0.0324, -0.0020),
                (0.0319, -0.0005),
                (0.0324, 0.0010),
                (0.0319, 0.0025),
                (0.0324, 0.0040),
                (0.0319, 0.0053),
                (0.0322, 0.0065),
            ],
            [
                (0.0274, -0.0065),
                (0.0274, 0.0065),
            ],
        ),
        material=rubber_black,
        name="zoom_grip",
    )
    zoom_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0325, length=0.018),
        mass=0.035,
        origin=Origin(rpy=(0.0, 0.0, 0.0)),
    )

    inner_carrier = model.part("inner_carrier")
    inner_carrier.visual(
        _shell_mesh(
            "inner_carrier_shell",
            [
                (0.0226, -0.0400),
                (0.0226, -0.0040),
                (0.0268, 0.0000),
                (0.0268, 0.0035),
                (0.0256, 0.0045),
            ],
            [
                (0.0207, -0.0400),
                (0.0207, 0.0045),
            ],
        ),
        material=satin_graphite,
        name="guide_sleeve",
    )
    inner_carrier.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0245, length=0.045),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, -0.017)),
    )

    inner_barrel = model.part("inner_barrel")
    inner_barrel.visual(
        _shell_mesh(
            "inner_barrel_shell",
            [
                (0.0238, 0.0000),
                (0.0238, 0.0060),
                (0.0248, 0.0080),
                (0.0248, 0.0180),
                (0.0260, 0.0200),
                (0.0260, 0.0300),
                (0.0265, 0.0316),
                (0.0265, 0.0330),
            ],
            [
                (0.0206, 0.0000),
                (0.0206, 0.0240),
                (0.0194, 0.0330),
            ],
        ),
        material=matte_black,
        name="inner_shell",
    )
    inner_barrel.visual(
        Cylinder(radius=0.0208, length=0.0040),
        origin=Origin(xyz=(0.0, 0.0, 0.0255)),
        material=satin_graphite,
        name="front_retainer",
    )
    inner_barrel.visual(
        Cylinder(radius=0.0192, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0278)),
        material=coated_glass,
        name="front_element",
    )
    inner_barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0265, length=0.033),
        mass=0.085,
        origin=Origin(xyz=(0.0, 0.0, 0.0165)),
    )

    model.articulation(
        "outer_to_zoom_ring",
        ArticulationType.REVOLUTE,
        parent=outer_barrel,
        child=zoom_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0200)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=2.5,
            lower=-0.8,
            upper=0.8,
        ),
    )
    model.articulation(
        "outer_to_inner_slide",
        ArticulationType.PRISMATIC,
        parent=outer_barrel,
        child=inner_carrier,
        origin=Origin(xyz=(0.0, 0.0, 0.0470)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=0.10,
            lower=0.0,
            upper=0.022,
        ),
    )
    model.articulation(
        "carrier_to_inner_barrel_spin",
        ArticulationType.REVOLUTE,
        parent=inner_carrier,
        child=inner_barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=2.0,
            lower=-0.75,
            upper=0.75,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    outer_barrel = object_model.get_part("outer_barrel")
    zoom_ring = object_model.get_part("zoom_ring")
    inner_carrier = object_model.get_part("inner_carrier")
    inner_barrel = object_model.get_part("inner_barrel")

    zoom_joint = object_model.get_articulation("outer_to_zoom_ring")
    slide_joint = object_model.get_articulation("outer_to_inner_slide")
    spin_joint = object_model.get_articulation("carrier_to_inner_barrel_spin")

    ctx.check(
        "all requested parts exist",
        all(part is not None for part in (outer_barrel, zoom_ring, inner_carrier, inner_barrel)),
    )
    ctx.check(
        "articulation axes follow the optical axis",
        zoom_joint.axis == (0.0, 0.0, 1.0)
        and slide_joint.axis == (0.0, 0.0, 1.0)
        and spin_joint.axis == (0.0, 0.0, 1.0),
        details=(
            f"zoom_axis={zoom_joint.axis}, slide_axis={slide_joint.axis}, "
            f"spin_axis={spin_joint.axis}"
        ),
    )

    ctx.allow_overlap(
        zoom_ring,
        outer_barrel,
        elem_a="zoom_grip",
        elem_b="outer_shell",
        reason=(
            "The zoom ring is intentionally represented as a close-running sleeve "
            "wrapped around the barrel waist, so the simplified coaxial shell fit "
            "is modeled as a nested overlap."
        ),
    )
    ctx.allow_overlap(
        inner_carrier,
        outer_barrel,
        elem_a="guide_sleeve",
        elem_b="outer_shell",
        reason=(
            "The retracting carrier is intentionally represented as a nested "
            "telescoping sleeve inside the outer housing, so the retained fit is "
            "modeled as a scoped overlap between the two shell proxies."
        ),
    )

    ctx.expect_within(
        inner_carrier,
        outer_barrel,
        axes="xy",
        inner_elem="guide_sleeve",
        outer_elem="outer_shell",
        margin=0.0,
        name="guide sleeve stays centered inside the outer barrel at rest",
    )
    ctx.expect_overlap(
        inner_carrier,
        outer_barrel,
        axes="z",
        elem_a="guide_sleeve",
        elem_b="outer_shell",
        min_overlap=0.020,
        name="collapsed guide sleeve remains deeply inserted",
    )
    ctx.expect_overlap(
        zoom_ring,
        outer_barrel,
        axes="z",
        elem_a="zoom_grip",
        elem_b="outer_shell",
        min_overlap=0.012,
        name="zoom ring sits over the outer barrel waist",
    )

    rest_position = ctx.part_world_position(inner_barrel)
    with ctx.pose({slide_joint: 0.022, spin_joint: 0.55, zoom_joint: 0.6}):
        ctx.expect_within(
            inner_carrier,
            outer_barrel,
            axes="xy",
            inner_elem="guide_sleeve",
            outer_elem="outer_shell",
            margin=0.0,
            name="guide sleeve stays centered at full extension",
        )
        ctx.expect_overlap(
            inner_carrier,
            outer_barrel,
            axes="z",
            elem_a="guide_sleeve",
            elem_b="outer_shell",
            min_overlap=0.015,
            name="extended guide sleeve retains insertion",
        )
        extended_position = ctx.part_world_position(inner_barrel)

    ctx.check(
        "inner barrel extends forward when the zoom stage telescopes",
        rest_position is not None
        and extended_position is not None
        and extended_position[2] > rest_position[2] + 0.015,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
