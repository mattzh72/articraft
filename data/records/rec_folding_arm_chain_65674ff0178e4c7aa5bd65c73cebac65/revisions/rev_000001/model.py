from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


HINGE_AXIS = (0.0, 1.0, 0.0)
HINGE_Z = 0.160
LINK_LENGTHS = (0.280, 0.260, 0.220)


def _axis_y() -> Origin:
    return Origin(rpy=(pi / 2.0, 0.0, 0.0))


def _axis_y_at(x: float, y: float = 0.0, z: float = 0.0) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(pi / 2.0, 0.0, 0.0))


def _add_clear_fork(part, *, x: float, z: float, material, prefix: str) -> None:
    """Add a transparent two-cheek hinge fork tied together by a rear bridge."""
    if prefix == "base":
        names = (("base_plate_0", "base_round_0"), ("base_plate_1", "base_round_1"))
    else:
        names = (("distal_plate_0", "distal_round_0"), ("distal_plate_1", "distal_round_1"))
    for sign, (plate_name, round_name) in zip((-1.0, 1.0), names):
        y = sign * 0.038
        part.visual(
            Box((0.095, 0.012, 0.092)),
            origin=Origin(xyz=(x, y, z)),
            material=material,
            name=plate_name,
        )
        part.visual(
            Cylinder(radius=0.034, length=0.014),
            origin=_axis_y_at(x, y, z),
            material=material,
            name=round_name,
        )


def _add_link(
    part,
    *,
    length: float,
    metal,
    clear,
    rubber=None,
    support_next: bool,
    end_pad: bool = False,
) -> None:
    # Central slim bar, cut short of both hinge centers so neighboring links
    # never occupy the same fork slot at the joint.
    part.visual(
        Box((length - 0.110, 0.032, 0.022)),
        origin=Origin(xyz=(length * 0.5, 0.0, 0.0)),
        material=metal,
        name="slim_bar",
    )

    # Proximal lug carried by this link; it fits between the parent's clear side
    # plates and gives the revolute origin a visible bearing boss.
    part.visual(
        Box((0.092, 0.034, 0.050)),
        origin=Origin(xyz=(0.025, 0.0, 0.0)),
        material=metal,
        name="prox_lug",
    )
    part.visual(
        Cylinder(radius=0.028, length=0.038),
        origin=_axis_y(),
        material=metal,
        name="prox_boss",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.064),
        origin=_axis_y(),
        material=metal,
        name="bearing_pin",
    )

    if support_next:
        # A rear cross block connects the clear fork plates to the bar while
        # leaving a visible gap for the next link's lug.
        part.visual(
            Box((0.046, 0.088, 0.030)),
            origin=Origin(xyz=(length - 0.065, 0.0, 0.0)),
            material=metal,
            name="fork_bridge",
        )
        _add_clear_fork(part, x=length, z=0.0, material=clear, prefix="distal")

    if end_pad:
        # Compact rubber pad at the distal end, with a short metal neck tied
        # into the slim bar.
        part.visual(
            Box((0.075, 0.048, 0.030)),
            origin=Origin(xyz=(length - 0.025, 0.0, 0.0)),
            material=metal,
            name="pad_neck",
        )
        part.visual(
            Box((0.090, 0.076, 0.032)),
            origin=Origin(xyz=(length + 0.045, 0.0, -0.002)),
            material=rubber,
            name="end_pad",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_top_folding_arm_chain")

    dark_metal = model.material("dark_anodized_metal", rgba=(0.10, 0.11, 0.12, 1.0))
    edge_metal = model.material("brushed_pin_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    clear_acrylic = model.material("clear_acrylic", rgba=(0.55, 0.82, 1.0, 0.34))
    rubber = model.material("matte_black_rubber", rgba=(0.025, 0.025, 0.025, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.340, 0.160, 0.022)),
        origin=Origin(xyz=(0.015, 0.0, 0.011)),
        material=dark_metal,
        name="base_foot",
    )
    base.visual(
        Box((0.090, 0.115, 0.014)),
        origin=Origin(xyz=(0.050, 0.0, 0.029)),
        material=edge_metal,
        name="table_wear_plate",
    )
    base.visual(
        Box((0.064, 0.090, 0.130)),
        origin=Origin(xyz=(-0.080, 0.0, 0.082)),
        material=dark_metal,
        name="upright_web",
    )
    base.visual(
        Box((0.054, 0.092, 0.050)),
        origin=Origin(xyz=(-0.065, 0.0, HINGE_Z - 0.057)),
        material=dark_metal,
        name="base_fork_bridge",
    )
    _add_clear_fork(base, x=0.0, z=HINGE_Z, material=clear_acrylic, prefix="base")

    link_0 = model.part("link_0")
    _add_link(link_0, length=LINK_LENGTHS[0], metal=dark_metal, clear=clear_acrylic, support_next=True)

    link_1 = model.part("link_1")
    _add_link(link_1, length=LINK_LENGTHS[1], metal=dark_metal, clear=clear_acrylic, support_next=True)

    distal_link = model.part("distal_link")
    _add_link(
        distal_link,
        length=LINK_LENGTHS[2],
        metal=dark_metal,
        clear=clear_acrylic,
        rubber=rubber,
        support_next=False,
        end_pad=True,
    )

    hinge_limits = MotionLimits(effort=8.0, velocity=2.2, lower=-1.55, upper=1.55)
    model.articulation(
        "base_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=link_0,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=HINGE_AXIS,
        motion_limits=hinge_limits,
    )
    model.articulation(
        "middle_hinge",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(LINK_LENGTHS[0], 0.0, 0.0)),
        axis=HINGE_AXIS,
        motion_limits=hinge_limits,
    )
    model.articulation(
        "distal_hinge",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=distal_link,
        origin=Origin(xyz=(LINK_LENGTHS[1], 0.0, 0.0)),
        axis=HINGE_AXIS,
        motion_limits=hinge_limits,
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    joints = [
        object_model.get_articulation("base_hinge"),
        object_model.get_articulation("middle_hinge"),
        object_model.get_articulation("distal_hinge"),
    ]
    ctx.check(
        "three_revolute_joints",
        all(j is not None and j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={joints!r}",
    )
    ctx.check(
        "hinge_axes_aligned",
        all(j is not None and tuple(round(v, 6) for v in j.axis) == HINGE_AXIS for j in joints),
        details=f"axes={[None if j is None else j.axis for j in joints]!r}",
    )

    base = object_model.get_part("base")
    link_0 = object_model.get_part("link_0")
    link_1 = object_model.get_part("link_1")
    distal_link = object_model.get_part("distal_link")
    ctx.expect_contact(
        base,
        link_0,
        elem_a="base_plate_0",
        elem_b="bearing_pin",
        contact_tol=0.001,
        name="first lug is captured by clear fork",
    )
    ctx.expect_contact(
        link_0,
        link_1,
        elem_a="distal_plate_0",
        elem_b="bearing_pin",
        contact_tol=0.001,
        name="second lug is captured by clear fork",
    )
    ctx.expect_contact(
        link_1,
        distal_link,
        elem_a="distal_plate_0",
        elem_b="bearing_pin",
        contact_tol=0.001,
        name="third lug is captured by clear fork",
    )
    ctx.expect_overlap(
        distal_link,
        distal_link,
        axes="x",
        elem_a="end_pad",
        elem_b="pad_neck",
        min_overlap=0.010,
        name="end pad is mounted to distal link",
    )

    with ctx.pose({"base_hinge": 0.45, "middle_hinge": -0.70, "distal_hinge": 0.55}):
        ctx.expect_origin_distance(
            "base",
            "distal_link",
            axes="xz",
            min_dist=0.25,
            name="folded pose keeps distal pad out from base",
        )

    return ctx.report()


object_model = build_object_model()
