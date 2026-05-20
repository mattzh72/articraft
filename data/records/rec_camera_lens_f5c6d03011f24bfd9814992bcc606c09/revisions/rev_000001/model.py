from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


def _shell_mesh_x(
    name: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 64,
):
    geom = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=segments,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="catadioptric_mirror_lens")

    body_black = model.material("body_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.15, 0.15, 0.16, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.16, 0.20, 0.22, 0.42))
    mirror_coat = model.material("mirror_coat", rgba=(0.82, 0.85, 0.88, 0.92))
    mount_metal = model.material("mount_metal", rgba=(0.48, 0.49, 0.52, 1.0))

    lens_housing = model.part("lens_housing")

    barrel_shell = _shell_mesh_x(
        "mirror_lens_barrel_shell",
        [
            (0.048, 0.000),
            (0.048, 0.008),
            (0.045, 0.014),
            (0.045, 0.062),
            (0.047, 0.070),
            (0.047, 0.078),
            (0.050, 0.086),
            (0.050, 0.092),
        ],
        [
            (0.020, 0.000),
            (0.020, 0.006),
            (0.040, 0.012),
            (0.040, 0.066),
            (0.041, 0.076),
            (0.040, 0.092),
        ],
    )
    focus_guide = _shell_mesh_x(
        "mirror_lens_focus_guide",
        [
            (0.0388, 0.016),
            (0.0388, 0.056),
        ],
        [
            (0.0370, 0.016),
            (0.0370, 0.056),
        ],
    )
    rear_guide_support = _shell_mesh_x(
        "mirror_lens_rear_guide_support",
        [
            (0.0402, 0.016),
            (0.0402, 0.019),
        ],
        [
            (0.0376, 0.016),
            (0.0376, 0.019),
        ],
    )
    front_guide_support = _shell_mesh_x(
        "mirror_lens_front_guide_support",
        [
            (0.0402, 0.053),
            (0.0402, 0.056),
        ],
        [
            (0.0376, 0.053),
            (0.0376, 0.056),
        ],
    )

    lens_housing.visual(barrel_shell, material=body_black, name="barrel_shell")
    lens_housing.visual(focus_guide, material=satin_black, name="focus_guide")
    lens_housing.visual(
        rear_guide_support,
        material=satin_black,
        name="rear_guide_support",
    )
    lens_housing.visual(
        front_guide_support,
        material=satin_black,
        name="front_guide_support",
    )
    lens_housing.visual(
        Cylinder(radius=0.026, length=0.009),
        origin=Origin(xyz=(-0.0045, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mount_metal,
        name="mount_ring",
    )
    lens_housing.visual(
        Cylinder(radius=0.040, length=0.0024),
        origin=Origin(xyz=(0.082, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_glass,
        name="corrector_glass",
    )
    lens_housing.visual(
        _shell_mesh_x(
            "mirror_lens_front_retainer",
            [
                (0.0408, 0.0808),
                (0.0408, 0.0832),
            ],
            [
                (0.0397, 0.0808),
                (0.0397, 0.0832),
            ],
        ),
        material=satin_black,
        name="front_retainer",
    )
    lens_housing.visual(
        Cylinder(radius=0.0145, length=0.0012),
        origin=Origin(xyz=(0.0816, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mirror_coat,
        name="secondary_spot",
    )
    lens_housing.visual(
        Box((0.016, 0.015, 0.011)),
        origin=Origin(xyz=(0.004, -0.031, -0.022)),
        material=body_black,
        name="filter_slot_block",
    )
    lens_housing.visual(
        Cylinder(radius=0.005, length=0.010),
        origin=Origin(xyz=(-0.004, -0.031, -0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="filter_knob_boss",
    )
    lens_housing.inertial = Inertial.from_geometry(
        Box((0.100, 0.100, 0.100)),
        mass=0.95,
        origin=Origin(xyz=(0.042, 0.0, 0.0)),
    )

    mirror_cell = model.part("mirror_cell")
    mirror_cell.visual(
        _shell_mesh_x(
            "mirror_cell_guide_sleeve",
            [
                (0.0370, -0.018),
                (0.0370, 0.010),
            ],
            [
                (0.0342, -0.018),
                (0.0342, 0.010),
            ],
        ),
        material=satin_black,
        name="guide_sleeve",
    )
    mirror_cell.visual(
        _shell_mesh_x(
            "mirror_cell_primary_annulus",
            [
                (0.0342, -0.018),
                (0.0342, -0.014),
            ],
            [
                (0.0115, -0.018),
                (0.0115, -0.014),
            ],
        ),
        material=mirror_coat,
        name="primary_mirror",
    )
    mirror_cell.visual(
        _shell_mesh_x(
            "mirror_cell_transfer_web",
            [
                (0.0342, 0.008),
                (0.0112, 0.015),
            ],
            [
                (0.0092, 0.008),
                (0.0082, 0.015),
            ],
        ),
        material=satin_black,
        name="transfer_web",
    )
    mirror_cell.visual(
        _shell_mesh_x(
            "mirror_cell_front_baffle",
            [
                (0.0105, 0.015),
                (0.0105, 0.0335),
            ],
            [
                (0.0082, 0.015),
                (0.0082, 0.0335),
            ],
        ),
        material=satin_black,
        name="front_baffle",
    )
    mirror_cell.inertial = Inertial.from_geometry(
        Cylinder(radius=0.037, length=0.052),
        mass=0.22,
        origin=Origin(xyz=(0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    filter_knob = model.part("filter_knob")
    filter_knob.visual(
        Cylinder(radius=0.0046, length=0.004),
        origin=Origin(xyz=(-0.002, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mount_metal,
        name="knob_hub",
    )
    filter_knob.visual(
        Cylinder(radius=0.0095, length=0.005),
        origin=Origin(xyz=(-0.0065, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="knob_wheel",
    )
    filter_knob.visual(
        Cylinder(radius=0.0108, length=0.0015),
        origin=Origin(xyz=(-0.0095, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_black,
        name="knob_rim",
    )
    filter_knob.inertial = Inertial.from_geometry(
        Cylinder(radius=0.011, length=0.012),
        mass=0.02,
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "focus_slide",
        ArticulationType.PRISMATIC,
        parent=lens_housing,
        child=mirror_cell,
        origin=Origin(xyz=(0.034, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.02,
            lower=0.0,
            upper=0.010,
        ),
    )
    model.articulation(
        "rear_filter_knob_spin",
        ArticulationType.CONTINUOUS,
        parent=lens_housing,
        child=filter_knob,
        origin=Origin(xyz=(-0.009, -0.031, -0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=10.0,
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

    lens_housing = object_model.get_part("lens_housing")
    mirror_cell = object_model.get_part("mirror_cell")
    filter_knob = object_model.get_part("filter_knob")
    focus_slide = object_model.get_articulation("focus_slide")
    knob_spin = object_model.get_articulation("rear_filter_knob_spin")

    ctx.allow_overlap(
        lens_housing,
        mirror_cell,
        elem_a="focus_guide",
        elem_b="guide_sleeve",
        reason=(
            "The primary mirror carrier is intentionally represented as a close "
            "sliding fit inside the internal focus guide sleeve."
        ),
    )

    ctx.check(
        "focus slide uses axial prismatic motion",
        focus_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(round(v, 5) for v in focus_slide.axis) == (1.0, 0.0, 0.0),
        details=f"type={focus_slide.articulation_type}, axis={focus_slide.axis}",
    )
    ctx.check(
        "filter knob uses continuous axial rotation",
        knob_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 5) for v in knob_spin.axis) == (1.0, 0.0, 0.0),
        details=f"type={knob_spin.articulation_type}, axis={knob_spin.axis}",
    )

    with ctx.pose({focus_slide: 0.0}):
        ctx.expect_within(
            mirror_cell,
            lens_housing,
            axes="yz",
            inner_elem="guide_sleeve",
            outer_elem="focus_guide",
            margin=0.001,
            name="mirror cell sleeve stays centered in rear guide at rest",
        )
        ctx.expect_overlap(
            mirror_cell,
            lens_housing,
            axes="x",
            elem_a="guide_sleeve",
            elem_b="focus_guide",
            min_overlap=0.024,
            name="mirror cell sleeve remains inserted in guide at rest",
        )
        ctx.expect_gap(
            lens_housing,
            mirror_cell,
            axis="x",
            positive_elem="corrector_glass",
            negative_elem="front_baffle",
            min_gap=0.003,
            name="front baffle clears the corrector glass at rest",
        )
        ctx.expect_contact(
            filter_knob,
            lens_housing,
            elem_a="knob_hub",
            elem_b="filter_knob_boss",
            contact_tol=1e-6,
            name="filter knob is mounted against its rear boss",
        )

    rest_pos = ctx.part_world_position(mirror_cell)
    with ctx.pose({focus_slide: 0.010, knob_spin: 1.7}):
        ctx.expect_within(
            mirror_cell,
            lens_housing,
            axes="yz",
            inner_elem="guide_sleeve",
            outer_elem="focus_guide",
            margin=0.001,
            name="mirror cell sleeve stays centered in rear guide when focused forward",
        )
        ctx.expect_overlap(
            mirror_cell,
            lens_housing,
            axes="x",
            elem_a="guide_sleeve",
            elem_b="focus_guide",
            min_overlap=0.024,
            name="mirror cell sleeve remains inserted in guide when focused forward",
        )
        ctx.expect_gap(
            lens_housing,
            mirror_cell,
            axis="x",
            positive_elem="corrector_glass",
            negative_elem="front_baffle",
            min_gap=0.003,
            name="front baffle still clears the corrector glass at max focus travel",
        )
        focused_pos = ctx.part_world_position(mirror_cell)

    ctx.check(
        "focus slide advances mirror cell forward",
        rest_pos is not None and focused_pos is not None and focused_pos[0] > rest_pos[0] + 0.008,
        details=f"rest={rest_pos}, focused={focused_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
