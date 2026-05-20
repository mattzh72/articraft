from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="solid_beam_rear_axle", assets=ASSETS)

    cast_iron = model.material("cast_iron", rgba=(0.28, 0.29, 0.31, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.50, 0.52, 0.55, 1.0))
    black_steel = model.material("black_steel", rgba=(0.18, 0.18, 0.19, 1.0))

    def add_x_cylinder(
        part,
        *,
        radius: float,
        length: float,
        center_x: float,
        name: str,
        material,
        y: float = 0.0,
        z: float = 0.0,
    ) -> None:
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=(center_x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=material,
            name=name,
        )

    def add_fixed(name: str, parent, child, xyz: tuple[float, float, float]) -> None:
        model.articulation(
            name,
            ArticulationType.FIXED,
            parent=parent,
            child=child,
            origin=Origin(xyz=xyz),
        )

    def build_axle_tube(part, sign: float) -> None:
        add_x_cylinder(
            part,
            radius=0.038,
            length=0.60,
            center_x=0.30 * sign,
            name="tube_shell",
            material=cast_iron,
        )
        add_x_cylinder(
            part,
            radius=0.043,
            length=0.10,
            center_x=0.55 * sign,
            name="flange_seat",
            material=cast_iron,
        )

    def build_flange(part, sign: float) -> None:
        add_x_cylinder(
            part,
            radius=0.052,
            length=0.060,
            center_x=0.030 * sign,
            name="hub_stub",
            material=machined_steel,
        )
        add_x_cylinder(
            part,
            radius=0.082,
            length=0.024,
            center_x=0.072 * sign,
            name="flange_disc",
            material=machined_steel,
        )
        add_x_cylinder(
            part,
            radius=0.028,
            length=0.014,
            center_x=0.090 * sign,
            name="pilot_boss",
            material=machined_steel,
        )
        for index in range(5):
            angle = math.pi / 2.0 + (2.0 * math.pi * index / 5.0)
            add_x_cylinder(
                part,
                radius=0.012,
                length=0.018,
                center_x=0.092 * sign,
                y=0.042 * math.cos(angle),
                z=0.042 * math.sin(angle),
                name=f"lug_{index + 1}",
                material=machined_steel,
            )

    def build_perch(part) -> None:
        part.visual(
            Box((0.16, 0.10, 0.04)),
            origin=Origin(xyz=(0.0, 0.0, 0.020)),
            material=black_steel,
            name="perch_block",
        )
        part.visual(
            Box((0.050, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, 0.032, 0.028)),
            material=black_steel,
            name="u_bolt_pad_front",
        )
        part.visual(
            Box((0.050, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, -0.032, 0.028)),
            material=black_steel,
            name="u_bolt_pad_rear",
        )

    carrier = model.part("carrier")
    add_x_cylinder(
        carrier,
        radius=0.042,
        length=0.30,
        center_x=0.0,
        name="center_bar",
        material=cast_iron,
    )

    pumpkin = model.part("pumpkin")
    pumpkin.visual(
        Cylinder(radius=0.078, length=0.11),
        origin=Origin(xyz=(0.010, 0.0, -0.045)),
        material=cast_iron,
        name="upper_neck",
    )
    pumpkin.visual(
        Sphere(radius=0.135),
        origin=Origin(xyz=(-0.010, 0.0, -0.130)),
        material=cast_iron,
        name="pumpkin_shell",
    )
    pumpkin.visual(
        Sphere(radius=0.095),
        origin=Origin(xyz=(0.015, 0.150, -0.115)),
        material=cast_iron,
        name="cover_bulge",
    )

    left_tube = model.part("left_tube")
    right_tube = model.part("right_tube")
    build_axle_tube(left_tube, -1.0)
    build_axle_tube(right_tube, 1.0)

    left_flange = model.part("left_flange")
    right_flange = model.part("right_flange")
    build_flange(left_flange, -1.0)
    build_flange(right_flange, 1.0)

    left_perch = model.part("left_perch")
    right_perch = model.part("right_perch")
    build_perch(left_perch)
    build_perch(right_perch)

    add_fixed("carrier_to_pumpkin", carrier, pumpkin, (0.0, 0.0, 0.0))
    add_fixed("carrier_to_left_tube", carrier, left_tube, (-0.149, 0.0, 0.0))
    add_fixed("carrier_to_right_tube", carrier, right_tube, (0.149, 0.0, 0.0))
    add_fixed("left_tube_to_left_flange", left_tube, left_flange, (-0.599, 0.0, 0.0))
    add_fixed("right_tube_to_right_flange", right_tube, right_flange, (0.599, 0.0, 0.0))
    add_fixed("left_tube_to_left_perch", left_tube, left_perch, (-0.28, 0.0, 0.037))
    add_fixed("right_tube_to_right_perch", right_tube, right_perch, (0.28, 0.0, 0.037))

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    carrier = object_model.get_part("carrier")
    pumpkin = object_model.get_part("pumpkin")
    left_tube = object_model.get_part("left_tube")
    right_tube = object_model.get_part("right_tube")
    left_flange = object_model.get_part("left_flange")
    right_flange = object_model.get_part("right_flange")
    left_perch = object_model.get_part("left_perch")
    right_perch = object_model.get_part("right_perch")

    center_bar = carrier.get_visual("center_bar")
    pumpkin_shell = pumpkin.get_visual("pumpkin_shell")
    cover_bulge = pumpkin.get_visual("cover_bulge")
    left_tube_shell = left_tube.get_visual("tube_shell")
    right_tube_shell = right_tube.get_visual("tube_shell")
    left_disc = left_flange.get_visual("flange_disc")
    right_disc = right_flange.get_visual("flange_disc")
    left_stub = left_flange.get_visual("hub_stub")
    right_stub = right_flange.get_visual("hub_stub")
    left_perch_block = left_perch.get_visual("perch_block")
    right_perch_block = right_perch.get_visual("perch_block")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_gap(
        carrier,
        left_tube,
        axis="x",
        max_gap=0.001,
        max_penetration=0.002,
        positive_elem=center_bar,
        negative_elem=left_tube_shell,
        name="left half-shaft tube seats against the center carrier",
    )
    ctx.expect_overlap(
        left_tube,
        carrier,
        axes="yz",
        min_overlap=0.070,
        elem_a=left_tube_shell,
        elem_b=center_bar,
        name="left tube stays coaxial with the carrier bar",
    )
    ctx.expect_gap(
        right_tube,
        carrier,
        axis="x",
        max_gap=0.001,
        max_penetration=0.002,
        positive_elem=right_tube_shell,
        negative_elem=center_bar,
        name="right half-shaft tube seats against the center carrier",
    )
    ctx.expect_overlap(
        right_tube,
        carrier,
        axes="yz",
        min_overlap=0.070,
        elem_a=right_tube_shell,
        elem_b=center_bar,
        name="right tube stays coaxial with the carrier bar",
    )

    ctx.expect_overlap(
        pumpkin,
        carrier,
        axes="xy",
        min_overlap=0.080,
        elem_a=pumpkin_shell,
        elem_b=center_bar,
        name="the differential pumpkin is centered under the beam",
    )
    ctx.expect_gap(
        pumpkin,
        carrier,
        axis="z",
        min_gap=-0.40,
        max_gap=-0.10,
        positive_elem=pumpkin_shell,
        negative_elem=center_bar,
        name="the pumpkin hangs distinctly below the axle centerline",
    )
    ctx.expect_gap(
        pumpkin,
        carrier,
        axis="y",
        min_gap=0.010,
        max_gap=0.050,
        positive_elem=cover_bulge,
        negative_elem=center_bar,
        name="the differential cover bulge is offset to one side of the beam center",
    )
    ctx.expect_overlap(
        pumpkin,
        pumpkin,
        axes="xz",
        min_overlap=0.120,
        elem_a=cover_bulge,
        elem_b=pumpkin_shell,
        name="the offset cover bulge remains blended into the main pumpkin casting",
    )

    ctx.expect_gap(
        left_tube,
        left_flange,
        axis="x",
        max_gap=0.001,
        max_penetration=0.002,
        positive_elem=left_tube_shell,
        negative_elem=left_stub,
        name="left wheel flange mounts at the left tube end",
    )
    ctx.expect_overlap(
        left_flange,
        left_tube,
        axes="yz",
        min_overlap=0.075,
        elem_a=left_stub,
        elem_b=left_tube_shell,
        name="left flange stays concentric with the left tube",
    )
    ctx.expect_gap(
        right_flange,
        right_tube,
        axis="x",
        max_gap=0.001,
        max_penetration=0.002,
        positive_elem=right_stub,
        negative_elem=right_tube_shell,
        name="right wheel flange mounts at the right tube end",
    )
    ctx.expect_overlap(
        right_flange,
        right_tube,
        axes="yz",
        min_overlap=0.075,
        elem_a=right_stub,
        elem_b=right_tube_shell,
        name="right flange stays concentric with the right tube",
    )

    ctx.expect_gap(
        left_perch,
        left_tube,
        axis="z",
        max_gap=0.001,
        max_penetration=0.002,
        positive_elem=left_perch_block,
        negative_elem=left_tube_shell,
        name="left spring perch sits on top of the left tube",
    )
    ctx.expect_overlap(
        left_perch,
        left_tube,
        axes="xy",
        min_overlap=0.075,
        elem_a=left_perch_block,
        elem_b=left_tube_shell,
        name="left spring perch spans the top of the left tube",
    )
    ctx.expect_gap(
        right_perch,
        right_tube,
        axis="z",
        max_gap=0.001,
        max_penetration=0.002,
        positive_elem=right_perch_block,
        negative_elem=right_tube_shell,
        name="right spring perch sits on top of the right tube",
    )
    ctx.expect_overlap(
        right_perch,
        right_tube,
        axes="xy",
        min_overlap=0.075,
        elem_a=right_perch_block,
        elem_b=right_tube_shell,
        name="right spring perch spans the top of the right tube",
    )

    for index in range(1, 6):
        left_lug = left_flange.get_visual(f"lug_{index}")
        right_lug = right_flange.get_visual(f"lug_{index}")

        ctx.expect_gap(
            left_flange,
            left_flange,
            axis="x",
            max_gap=0.001,
            max_penetration=0.002,
            positive_elem=left_disc,
            negative_elem=left_lug,
            name=f"left flange lug boss {index} stands proud from the wheel flange face",
        )
        ctx.expect_overlap(
            left_flange,
            left_flange,
            axes="yz",
            min_overlap=0.020,
            elem_a=left_lug,
            elem_b=left_disc,
            name=f"left flange lug boss {index} stays on the left flange bolt circle",
        )

        ctx.expect_gap(
            right_flange,
            right_flange,
            axis="x",
            max_gap=0.001,
            max_penetration=0.002,
            positive_elem=right_lug,
            negative_elem=right_disc,
            name=f"right flange lug boss {index} stands proud from the wheel flange face",
        )
        ctx.expect_overlap(
            right_flange,
            right_flange,
            axes="yz",
            min_overlap=0.020,
            elem_a=right_lug,
            elem_b=right_disc,
            name=f"right flange lug boss {index} stays on the right flange bolt circle",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
