from __future__ import annotations

import math

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
    model = ArticulatedObject(name="four_joint_revolute_chain")

    link_pitch = 0.34
    hinge_radius = 0.040
    hinge_height = 0.075
    link_width = 0.055
    link_thickness = 0.034
    bar_overlap = 0.003

    blue = Material("anodized_blue", rgba=(0.10, 0.28, 0.82, 1.0))
    orange = Material("safety_orange", rgba=(0.95, 0.43, 0.08, 1.0))
    hinge_metal = Material("brushed_hinge_metal", rgba=(0.58, 0.60, 0.62, 1.0))
    pin_dark = Material("dark_pin_caps", rgba=(0.04, 0.045, 0.05, 1.0))
    base_dark = Material("powder_coated_base", rgba=(0.11, 0.12, 0.13, 1.0))

    def add_link_geometry(part, *, bar_material, prefix: str, terminal_cap: bool = False) -> None:
        """A link frame sits at the proximal hinge axis and points along +X."""
        part.visual(
            Cylinder(radius=hinge_radius, length=hinge_height),
            origin=Origin(),
            material=hinge_metal,
            name=f"{prefix}_hinge_block",
        )
        part.visual(
            Cylinder(radius=hinge_radius * 0.42, length=hinge_height + 0.018),
            origin=Origin(),
            material=pin_dark,
            name=f"{prefix}_hinge_pin",
        )

        bar_start = hinge_radius - bar_overlap
        bar_end = link_pitch - hinge_radius
        if terminal_cap:
            bar_end = link_pitch + bar_overlap
        bar_length = bar_end - bar_start
        part.visual(
            Box((bar_length, link_width, link_thickness)),
            origin=Origin(xyz=(bar_start + bar_length / 2.0, 0.0, 0.0)),
            material=bar_material,
            name=f"{prefix}_bar",
        )

        if terminal_cap:
            part.visual(
                Cylinder(radius=hinge_radius * 0.84, length=hinge_height * 0.78),
                origin=Origin(xyz=(link_pitch, 0.0, 0.0)),
                material=bar_material,
                name=f"{prefix}_end_cap",
            )

    base_link = model.part("base_link")
    base_link.visual(
        Box((0.36, 0.24, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, -0.205)),
        material=base_dark,
        name="base_foot",
    )
    base_link.visual(
        Box((0.086, 0.115, 0.225)),
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
        material=base_dark,
        name="base_post",
    )
    add_link_geometry(base_link, bar_material=blue, prefix="base")

    links = [base_link]
    for index in range(1, 5):
        link = model.part(f"link_{index}")
        add_link_geometry(
            link,
            bar_material=orange if index % 2 else blue,
            prefix=f"link_{index}",
            terminal_cap=(index == 4),
        )
        links.append(link)

    limits = MotionLimits(
        effort=18.0,
        velocity=2.4,
        lower=-math.pi / 2.0,
        upper=math.pi / 2.0,
    )
    for index in range(4):
        model.articulation(
            f"hinge_{index}",
            ArticulationType.REVOLUTE,
            parent=links[index],
            child=links[index + 1],
            origin=Origin(xyz=(link_pitch, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=limits,
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

    hinges = [object_model.get_articulation(f"hinge_{index}") for index in range(4)]
    links = [object_model.get_part("base_link")] + [
        object_model.get_part(f"link_{index}") for index in range(1, 5)
    ]

    ctx.check(
        "four revolute joints",
        len(hinges) == 4
        and all(hinge.articulation_type == ArticulationType.REVOLUTE for hinge in hinges),
        details=f"found {[hinge.articulation_type for hinge in hinges]}",
    )
    ctx.check(
        "all hinge axes parallel",
        all(tuple(hinge.axis) == (0.0, 0.0, 1.0) for hinge in hinges),
        details=f"axes {[hinge.axis for hinge in hinges]}",
    )
    ctx.check(
        "ninety degree bidirectional travel",
        all(
            hinge.motion_limits is not None
            and abs(hinge.motion_limits.lower + math.pi / 2.0) < 1e-6
            and abs(hinge.motion_limits.upper - math.pi / 2.0) < 1e-6
            for hinge in hinges
        ),
        details="each hinge should travel from -pi/2 to +pi/2",
    )

    for index, hinge in enumerate(hinges):
        child = links[index + 1]
        ctx.expect_origin_gap(
            child,
            links[index],
            axis="x",
            min_gap=0.339,
            max_gap=0.341,
            name=f"hinge_{index} spacing",
        )
        ctx.expect_contact(
            links[index],
            child,
            elem_a=f"{'base' if index == 0 else f'link_{index}'}_bar",
            elem_b=f"link_{index + 1}_hinge_block",
            contact_tol=0.002,
            name=f"hinge_{index} block touches parent link",
        )

    rest_end = ctx.part_element_world_aabb(links[4], elem="link_4_end_cap")
    with ctx.pose({"hinge_3": math.pi / 2.0}):
        turned_end = ctx.part_element_world_aabb(links[4], elem="link_4_end_cap")
    ctx.check(
        "last link rotates in chain plane",
        rest_end is not None
        and turned_end is not None
        and turned_end[0][1] > rest_end[0][1] + 0.20,
        details=f"rest={rest_end}, turned={turned_end}",
    )

    return ctx.report()


object_model = build_object_model()
