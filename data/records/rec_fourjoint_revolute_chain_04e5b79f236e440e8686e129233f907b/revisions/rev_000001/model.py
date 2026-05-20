from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_top_revolute_chain")

    dark_steel = Material("dark_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.55, 0.57, 0.56, 1.0))
    clear_plate = Material("clear_acrylic", rgba=(0.60, 0.88, 1.00, 0.38))
    rubber = Material("matte_rubber", rgba=(0.025, 0.025, 0.025, 1.0))

    base_joint_z = 0.17
    link_length = 0.16
    hub_radius = 0.026
    hub_length = 0.086
    plate_y = 0.049
    plate_thickness = 0.012

    def y_cylinder(radius: float, length: float) -> Cylinder:
        return Cylinder(radius=radius, length=length)

    cylinder_to_y = Origin(rpy=(pi / 2.0, 0.0, 0.0))

    def add_fork(part, joint_z: float, *, base_scale: float = 1.0) -> None:
        """Parent-side fork: clear plates plus a local hinge support block."""
        part.visual(
            Box((0.082 * base_scale, 0.100, 0.030)),
            origin=Origin(xyz=(0.0, 0.0, joint_z - 0.065)),
            material=dark_steel,
            name="fork_bridge",
        )
        for side, y in (("pos", plate_y), ("neg", -plate_y)):
            part.visual(
                Box((0.086 * base_scale, plate_thickness, 0.125)),
                origin=Origin(xyz=(0.0, y, joint_z)),
                material=clear_plate,
                name=f"joint_plate_{side}",
            )
            part.visual(
                y_cylinder(radius=0.018, length=0.014),
                origin=Origin(xyz=(0.0, y + (0.012 if y > 0.0 else -0.012), joint_z), rpy=(pi / 2.0, 0.0, 0.0)),
                material=brushed_steel,
                name=f"pin_cap_{side}",
            )

    base = model.part("base")
    base.visual(
        Box((0.34, 0.22, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=dark_steel,
        name="foot",
    )
    base.visual(
        Box((0.27, 0.15, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=brushed_steel,
        name="top_plate",
    )
    base.visual(
        Box((0.090, 0.095, 0.106)),
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        material=dark_steel,
        name="pedestal",
    )
    add_fork(base, base_joint_z, base_scale=1.12)

    def add_link(part, *, has_fork: bool, has_end_tab: bool) -> None:
        part.visual(
            y_cylinder(radius=hub_radius, length=hub_length),
            origin=cylinder_to_y,
            material=brushed_steel,
            name="proximal_hub",
        )
        bar_bottom = 0.018
        bar_top = link_length - (0.055 if has_fork else 0.013)
        part.visual(
            Box((0.034, 0.030, bar_top - bar_bottom)),
            origin=Origin(xyz=(0.0, 0.0, (bar_top + bar_bottom) / 2.0)),
            material=dark_steel,
            name="link_bar",
        )
        part.visual(
            Box((0.050, 0.040, 0.020)),
            origin=Origin(xyz=(0.0, 0.0, 0.034)),
            material=dark_steel,
            name="hub_neck",
        )
        if has_fork:
            add_fork(part, link_length)
        if has_end_tab:
            part.visual(
                Box((0.078, 0.052, 0.026)),
                origin=Origin(xyz=(0.0, 0.0, link_length)),
                material=brushed_steel,
                name="end_tab",
            )
            part.visual(
                y_cylinder(radius=0.013, length=0.060),
                origin=Origin(xyz=(0.0, 0.0, link_length), rpy=(pi / 2.0, 0.0, 0.0)),
                material=dark_steel,
                name="tab_pin",
            )

    links = []
    for idx in range(4):
        link = model.part(f"link_{idx}")
        add_link(link, has_fork=idx < 3, has_end_tab=idx == 3)
        links.append(link)

    limits = MotionLimits(effort=8.0, velocity=2.0, lower=-1.35, upper=1.35)
    model.articulation(
        "joint_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=links[0],
        origin=Origin(xyz=(0.0, 0.0, base_joint_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=limits,
    )
    for idx in range(1, 4):
        model.articulation(
            f"joint_{idx}",
            ArticulationType.REVOLUTE,
            parent=links[idx - 1],
            child=links[idx],
            origin=Origin(xyz=(0.0, 0.0, link_length)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    joints = [object_model.get_articulation(f"joint_{idx}") for idx in range(4)]
    links = [object_model.get_part(f"link_{idx}") for idx in range(4)]
    base = object_model.get_part("base")

    ctx.check(
        "four revolute joints in sequence",
        len(object_model.articulations) == 4
        and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )
    ctx.check(
        "all hinge axes share the motion plane",
        all(tuple(round(v, 6) for v in j.axis) == (0.0, 1.0, 0.0) for j in joints),
        details=f"axes={[j.axis for j in joints]}",
    )

    ctx.expect_gap(
        base,
        links[0],
        axis="y",
        positive_elem="joint_plate_pos",
        negative_elem="proximal_hub",
        min_gap=0.0,
        max_gap=0.002,
        name="first hub clears positive side plate",
    )
    ctx.expect_gap(
        links[0],
        base,
        axis="y",
        positive_elem="proximal_hub",
        negative_elem="joint_plate_neg",
        min_gap=0.0,
        max_gap=0.002,
        name="first hub clears negative side plate",
    )
    ctx.expect_within(
        links[1],
        links[0],
        axes="xz",
        inner_elem="proximal_hub",
        outer_elem="joint_plate_pos",
        margin=0.002,
        name="second hub sits inside the fork outline",
    )

    rest_pos = ctx.part_world_position(links[1])
    with ctx.pose({"joint_0": 0.55}):
        posed_pos = ctx.part_world_position(links[1])
    ctx.check(
        "first joint swings the downstream stack",
        rest_pos is not None
        and posed_pos is not None
        and abs(posed_pos[0] - rest_pos[0]) > 0.050,
        details=f"rest={rest_pos}, posed={posed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
